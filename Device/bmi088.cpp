#include "bmi088.h"

#include "bmi088_driver.h"
#include "MahonyAHRS.h"
#include "common/alg_common.h"
#include "estimation/QuaternionEKF.h"
#include "imu_topics.hpp"
#include "mcu_topics.hpp"

#include "tim.h"

#include <array>
#include <cmath>
#include <cstdint>

namespace {
constexpr uint32_t kImuPeriodMs = 1;

// 控制策略沿用旧实现：
// 1) 上电先做 gyro bias 平均（kGyroBiasAvgSamples 次）
// 2) 然后等待温度接近 RefTemp 并稳定 (kTempStableTicks)
// 3) 最后进入 Running：更新 EKF/Mahony 并发布 topics
constexpr std::uint32_t kGyroBiasAvgSamples = 1000;
constexpr std::uint32_t kTempStableTicks = 300;

constexpr float kRefTempC = 40.0f;

constexpr bool kCheatMode = true;
constexpr float kCheatYawGyroAbsThreshold = 0.003f;
}

Bmi088::Bmi088()
    : drv_(Bmi088Drv::Instance())
{
}

void Bmi088::AlgoInit()
{
    alg::QuaternionEkf::Params params{};
    params.process_noise_quat_ = 10.0f;
    params.process_noise_gyro_bias_ = 0.001f;
    params.measure_noise_accel_ = 10000000.0f;
    params.fading_lambda_ = 1.0f;
    params.dt_ = 0.001f;
    params.accel_lpf_coef_ = 0.0f;
    quat_ekf_.Init(params);

    mahony_.Init(1000);
}

void Bmi088::HwInit()
{
    {
        alg::DjiPidConfig cfg{};
        cfg.kp = 1600.0f;
        cfg.ki = 50.0f;
        cfg.kd = 40.0f;
        cfg.max_out = 2000.0f;
        cfg.max_iout = 200.0f;
        cfg.mode = alg::PID_POSITION;
        temperature_pid_.configure(cfg);
    }

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // BMI088 init (0 means OK in this codebase)
    while (drv_.Init()) {
    }
}

void Bmi088::TemperatureCtrlStep()
{
    const float out = temperature_pid_.update(kRefTempC, temp_c_);

    uint32_t pwm = static_cast<uint32_t>(out);
    if (out < 0.0f) {
        pwm = 0;
    }
    htim3.Instance->CCR4 = pwm;
}

void Bmi088::Publish(float pitch_deg,
                     float yaw_deg,
                     float yaw_total_deg,
                     float yaw_omega_rad_s,
                     float pitch_omega_rad_s,
                     const float q_vision[4])
{
    {
        orb::ImuData out{};
        out.q[0] = q_vision[0];
        out.q[1] = q_vision[1];
        out.q[2] = q_vision[2];
        out.q[3] = q_vision[3];
        out.yaw_angle = -yaw_deg;
        out.pitch_angle = -pitch_deg;
        out.yaw_omega = yaw_omega_rad_s;
        out.pitch_omega = pitch_omega_rad_s;
        orb::imu_data.publish(out);
    }

    {
        orb::McuImu fb{};
        fb.yaw_total_angle_f = yaw_total_deg;
        fb.yaw_omega_f = yaw_omega_rad_s;
        fb.pitch_f = pitch_deg;
        fb.pitch_omega_f = pitch_omega_rad_s;
        orb::mcu_imu.publish(fb);
    }
}

bool Bmi088::Start()
{
    if (started_) {
        configASSERT(false);
        return false;
    }

    started_ = true;

    const osThreadAttr_t attr{
        .name = "imu_sample",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t)osPriorityNormal,
    };

    thread_ = osThreadNew(&Bmi088::TaskEntry, this, &attr);
    if (!thread_) {
        configASSERT(false);
        return false;
    }

    return true;
}

void Bmi088::TaskEntry(void* arg)
{
    static_cast<Bmi088*>(arg)->Task();
}

void Bmi088::Task()
{
    AlgoInit();
    HwInit();

    for (;;) {
        // loop counter
        ++loop_count_;

        // 1) read BMI088
        drv_.Read(gyro_, accel_, temp_c_);

        // 2) one-shot Mahony quick init (from accel)
        if (!first_mahony_done_) {
            first_mahony_done_ = true;
            mahony_.InitFromAccMag(accel_[0], accel_[1], accel_[2], 0, 0, 0);
        }

        // 3) Stage machine
        if (stage_ == Stage::BiasCalibrating) {
            gyro_bias_sum_[0] += gyro_[0];
            gyro_bias_sum_[1] += gyro_[1];
            gyro_bias_sum_[2] += gyro_[2];
            bias_samples_++;

            if (bias_samples_ >= kGyroBiasAvgSamples) {
                const float inv_n = 1.0f / static_cast<float>(bias_samples_);
                gyro_bias_[0] = gyro_bias_sum_[0] * inv_n;
                gyro_bias_[1] = gyro_bias_sum_[1] * inv_n;
                gyro_bias_[2] = gyro_bias_sum_[2] * inv_n;
                stage_ = Stage::WarmUp;
            }
        } else if (stage_ == Stage::WarmUp) {
            // 温度稳定判断
            if (std::fabs(temp_c_ - kRefTempC) < 0.5f) {
                temp_stable_ticks_++;
                if (temp_stable_ticks_ > kTempStableTicks) {
                    stage_ = Stage::Running;
                }
            } else {
                temp_stable_ticks_ = 0;
            }
        } else {
            // Running: update attitude
            float gx = gyro_[0] - gyro_bias_[0];
            float gy = gyro_[1] - gyro_bias_[1];
            float gz = gyro_[2] - gyro_bias_[2];

            if constexpr (kCheatMode) {
                if (std::fabs(gz) < kCheatYawGyroAbsThreshold) {
                    gz = 0.0f;
                }
            }

            quat_ekf_.Update(gx, gy, gz, accel_[0], accel_[1], accel_[2]);
            mahony_.Update(gx, gy, gz, accel_[0], accel_[1], accel_[2], 0, 0, 0);
            mahony_.ComputeAngles();

            const float roll_deg = quat_ekf_.RollDeg();
            const float pitch_deg = quat_ekf_.PitchDeg();
            const float yaw_deg = quat_ekf_.YawDeg();

            const float yaw_total_deg = quat_ekf_.YawTotalDeg();
            const float yaw_omega_rad_s = quat_ekf_.YawOmegaRad();
            const float pitch_omega_rad_s = quat_ekf_.PitchOmegaRad();

            // 给视觉：重构四元数（pitch 取反）
            const float phi = roll_deg * DEG_TO_RAD;
            float theta = pitch_deg * DEG_TO_RAD;
            const float psi = yaw_deg * DEG_TO_RAD;
            theta = -theta;

            const float cy = cosf(psi * 0.5f);
            const float sy = sinf(psi * 0.5f);
            const float cp = cosf(theta * 0.5f);
            const float sp = sinf(theta * 0.5f);
            const float cr = cosf(phi * 0.5f);
            const float sr = sinf(phi * 0.5f);

            float q_vision[4] = {0};
            q_vision[0] = cy * cp * cr + sy * sp * sr;
            q_vision[1] = cy * cp * sr - sy * sp * cr;
            q_vision[2] = cy * sp * cr + sy * cp * sr;
            q_vision[3] = sy * cp * cr - cy * sp * sr;

            Publish(pitch_deg, yaw_deg, yaw_total_deg, yaw_omega_rad_s, pitch_omega_rad_s, q_vision);
        }

        // 4) temperature control @100Hz
        if ((loop_count_ % 10) == 0) {
            TemperatureCtrlStep();
        }

        osDelay(pdMS_TO_TICKS(kImuPeriodMs));
    }
}

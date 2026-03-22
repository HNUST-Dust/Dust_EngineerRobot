#include "dvc_servo.h"
#include <cstring>
#include <algorithm>
#include "cmsis_os2.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

// If multiple ServoBus instances are used on different UARTs, you can expand this map.
static ServoBus *g_servo_instance_for_huart1 = nullptr;

// Protocol constants
static constexpr uint8_t kFrameHead = 0x55;

ServoBus::ServoBus(UART_HandleTypeDef *huart)
    : huart_(huart)
{
}

void ServoBus::Init(UART_HandleTypeDef *huart)
{
    huart_ = huart;

}

uint8_t ServoBus::SendPacket(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t params_len)
{
    // Packet: 0x55 0x55 ID Length Cmd [params] Checksum
    const uint8_t length = 1 + params_len; // Cmd + params
    const size_t total = 2 + 1 + 1 + length + 1; // head*2 + ID + Length + (Cmd+params) + checksum
    std::array<uint8_t, 64> buf{};
    if (total > buf.size()) return 0;

    size_t idx = 0;
    buf[idx++] = kFrameHead;
    buf[idx++] = kFrameHead;
    buf[idx++] = id;
    buf[idx++] = length;
    buf[idx++] = cmd;
    uint16_t sum = id + length + cmd;
    if (params_len && params) {
        std::memcpy(&buf[idx], params, params_len);
        for (uint8_t i = 0; i < params_len; ++i) sum += params[i];
        idx += params_len;
    }
    const uint8_t checksum = static_cast<uint8_t>(~(sum & 0xFF));
    buf[idx++] = checksum;

    // send using existing bsp helper
    if (huart_ != nullptr) {
        // 无 DE 引脚时：在发送前停止接收，使用 DMA 异步发送，
        // 在 HAL_UART_TxCpltCallback 中恢复接收（见 bsp_usart.cpp）
        HAL_UART_AbortReceive(huart_);
        // 小延时让硬件稳定
        osDelay(1);
        // clear tx flag for this uart
        if (huart_->Instance == USART1) g_uart1_manage_object.tx_cplt_flag = false;
        else if (huart_->Instance == USART2) g_uart2_manage_object.tx_cplt_flag = false;
        else if (huart_->Instance == USART3) g_uart3_manage_object.tx_cplt_flag = false;
        else if (huart_->Instance == UART4) g_uart4_manage_object.tx_cplt_flag = false;
        else if (huart_->Instance == UART5) g_uart5_manage_object.tx_cplt_flag = false;
        else if (huart_->Instance == USART6) g_uart6_manage_object.tx_cplt_flag = false;
        else if (huart_->Instance == UART7) g_uart7_manage_object.tx_cplt_flag = false;
        else if (huart_->Instance == UART8) g_uart8_manage_object.tx_cplt_flag = false;
        else if (huart_->Instance == USART10) g_uart10_manage_object.tx_cplt_flag = false;

        if (HAL_UART_Transmit_DMA(huart_, buf.data(), static_cast<uint16_t>(idx)) != HAL_OK) {
            // 发送失败，尝试恢复接收
            uart_reinit(huart_);
            return 0;
        }
        // 等待 DMA 发送完成标志（tx_cplt_flag 在 HAL_UART_TxCpltCallback 中被置位）
        const uint32_t start = HAL_GetTick();
        const uint32_t timeout = 200; // ms
        bool ok = false;
        while ((HAL_GetTick() - start) < timeout) {
            if (huart_->Instance == USART1 && g_uart1_manage_object.tx_cplt_flag) { ok = true; break; }
            if (huart_->Instance == USART2 && g_uart2_manage_object.tx_cplt_flag) { ok = true; break; }
            if (huart_->Instance == USART3 && g_uart3_manage_object.tx_cplt_flag) { ok = true; break; }
            if (huart_->Instance == UART4 && g_uart4_manage_object.tx_cplt_flag) { ok = true; break; }
            if (huart_->Instance == UART5 && g_uart5_manage_object.tx_cplt_flag) { ok = true; break; }
            if (huart_->Instance == USART6 && g_uart6_manage_object.tx_cplt_flag) { ok = true; break; }
            if (huart_->Instance == UART7 && g_uart7_manage_object.tx_cplt_flag) { ok = true; break; }
            if (huart_->Instance == UART8 && g_uart8_manage_object.tx_cplt_flag) { ok = true; break; }
            if (huart_->Instance == USART10 && g_uart10_manage_object.tx_cplt_flag) { ok = true; break; }
            osDelay(1);
        }
        if (!ok) {
            // 超时仍未完成，尝试重启接收
            uart_reinit(huart_);
            return 0;
        }
        return 1;
    }

    // fallback: use existing helper if no huart configured
    uart_send_data(huart_, buf.data(), static_cast<uint16_t>(idx));
    return 1;
}

uint16_t ServoBus::AngleToRaw(float degrees)
{
    // Map 0..240 deg -> 0..1000
    const float clamped = std::min(std::max(degrees, 0.0f), 240.0f);
    const uint16_t raw = static_cast<uint16_t>(clamped * (1000.0f / 240.0f) + 0.5f);
    return raw;
}

bool ServoBus::MoveToAngle(uint8_t id, float degrees, uint32_t time_ms)
{
    // Cmd 1: SERVO_MOVE_TIME_WRITE, Length 7 -> params: angleL angleH timeL timeH
    uint16_t raw = AngleToRaw(degrees);
    uint8_t params[4];
    params[0] = static_cast<uint8_t>(raw & 0xFF);
    params[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    params[2] = static_cast<uint8_t>(time_ms & 0xFF);
    params[3] = static_cast<uint8_t>((time_ms >> 8) & 0xFF);
    return SendPacket(id, 1 /*cmd*/, params, sizeof(params)) != 0;
}

bool ServoBus::SetSpeed(uint8_t id, int16_t speed)
{
    // Cmd 29: SERVO_OR_MOTOR_MODE_WRITE, Length 7
    // params: mode(1= motor) reserve(0) speedL speedH
    uint8_t params[4];
    params[0] = 1; // motor mode
    params[1] = 0;
    // protocol transmits signed short as unsigned bytes (little endian)
    params[2] = static_cast<uint8_t>(static_cast<uint16_t>(speed) & 0xFF);
    params[3] = static_cast<uint8_t>((static_cast<uint16_t>(speed) >> 8) & 0xFF);
    return SendPacket(id, 29 /*cmd*/, params, sizeof(params)) != 0;
}

bool ServoBus::Stop(uint8_t id)
{
    return SendPacket(id, 12 /*SERVO_MOVE_STOP*/, nullptr, 0) != 0;
}

bool ServoBus::MoveStart(uint8_t id)
{
    return SendPacket(id, 11 /*SERVO_MOVE_START*/, nullptr, 0) != 0;
}

// Generic response state for read commands
static volatile bool s_resp_ready = false;
static volatile uint8_t s_resp_cmd = 0;
static volatile uint8_t s_resp_id = 0;
static volatile uint8_t s_resp_payload_len = 0;
static uint8_t s_resp_payload[16] = {0};

// helper to wait for a response with expected cmd
static bool WaitForResponse(uint8_t expected_cmd, uint32_t timeout_ms)
{
    s_resp_ready = false;
    s_resp_cmd = 0;
    s_resp_payload_len = 0;
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (s_resp_ready && s_resp_cmd == expected_cmd) return true;
        osDelay(1);
    }
    return false;
}

uint8_t ServoBus::ReadID(uint8_t query_id, uint32_t timeout_ms)
{
    SendPacket(query_id, 14, nullptr, 0);
    if (!WaitForResponse(14, timeout_ms)) return 0xFF;
    return static_cast<uint8_t>(s_resp_payload_len >= 1 ? s_resp_payload[0] : 0xFF);
}

bool ServoBus::WriteID(uint8_t current_id, uint8_t new_id)
{
    uint8_t params[1] = { new_id };
    if (SendPacket(current_id, 13 /*SERVO_ID_WRITE*/, params, 1) == 0) return false;
    // 等待舵机内部保存完毕并使用新 ID 生效，尝试多次读取确认
    const int kRetries = 3;
    const uint32_t kDelayMs = 120;
    for (int i = 0; i < kRetries; ++i) {
        osDelay(pdMS_TO_TICKS(kDelayMs));
        uint8_t resp = ReadID(new_id, 200);
        if (resp == new_id) return true;
    }
    return false;
}

bool ServoBus::MoveTimeWait(uint8_t id, float degrees, uint32_t time_ms)
{
    uint16_t raw = AngleToRaw(degrees);
    uint8_t params[4];
    params[0] = static_cast<uint8_t>(raw & 0xFF);
    params[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    params[2] = static_cast<uint8_t>(time_ms & 0xFF);
    params[3] = static_cast<uint8_t>((time_ms >> 8) & 0xFF);
    return SendPacket(id, 7 /*SERVO_MOVE_TIME_WAIT_WRITE*/, params, 4) != 0;
}

bool ServoBus::ReadMoveTime(uint8_t id, uint16_t &out_angle_raw, uint32_t &out_time_ms, uint32_t timeout_ms)
{
    SendPacket(id, 2, nullptr, 0);
    if (!WaitForResponse(2, timeout_ms)) return false;
    if (s_resp_payload_len < 4) return false;
    out_angle_raw = static_cast<uint16_t>(s_resp_payload[0]) | (static_cast<uint16_t>(s_resp_payload[1]) << 8);
    out_time_ms = static_cast<uint32_t>(s_resp_payload[2]) | (static_cast<uint32_t>(s_resp_payload[3]) << 8);
    return true;
}

bool ServoBus::ReadMoveTimeWait(uint8_t id, uint16_t &out_angle_raw, uint32_t &out_time_ms, uint32_t timeout_ms)
{
    SendPacket(id, 8, nullptr, 0);
    if (!WaitForResponse(8, timeout_ms)) return false;
    if (s_resp_payload_len < 4) return false;
    out_angle_raw = static_cast<uint16_t>(s_resp_payload[0]) | (static_cast<uint16_t>(s_resp_payload[1]) << 8);
    out_time_ms = static_cast<uint32_t>(s_resp_payload[2]) | (static_cast<uint32_t>(s_resp_payload[3]) << 8);
    return true;
}

bool ServoBus::AngleOffsetAdjust(uint8_t id, int8_t offset)
{
    uint8_t params[1] = { static_cast<uint8_t>(offset) };
    return SendPacket(id, 17, params, 1) != 0;
}

bool ServoBus::AngleOffsetWrite(uint8_t id)
{
    return SendPacket(id, 18, nullptr, 0) != 0;
}

bool ServoBus::AngleOffsetRead(uint8_t id, int8_t &out_offset, uint32_t timeout_ms)
{
    SendPacket(id, 19, nullptr, 0);
    if (!WaitForResponse(19, timeout_ms)) return false;
    if (s_resp_payload_len < 1) return false;
    out_offset = static_cast<int8_t>(s_resp_payload[0]);
    return true;
}

bool ServoBus::AngleLimitWrite(uint8_t id, uint16_t min_raw, uint16_t max_raw)
{
    uint8_t params[4];
    params[0] = static_cast<uint8_t>(min_raw & 0xFF);
    params[1] = static_cast<uint8_t>((min_raw >> 8) & 0xFF);
    params[2] = static_cast<uint8_t>(max_raw & 0xFF);
    params[3] = static_cast<uint8_t>((max_raw >> 8) & 0xFF);
    return SendPacket(id, 20, params, 4) != 0;
}

bool ServoBus::AngleLimitRead(uint8_t id, uint16_t &out_min_raw, uint16_t &out_max_raw, uint32_t timeout_ms)
{
    SendPacket(id, 21, nullptr, 0);
    if (!WaitForResponse(21, timeout_ms)) return false;
    if (s_resp_payload_len < 4) return false;
    out_min_raw = static_cast<uint16_t>(s_resp_payload[0]) | (static_cast<uint16_t>(s_resp_payload[1]) << 8);
    out_max_raw = static_cast<uint16_t>(s_resp_payload[2]) | (static_cast<uint16_t>(s_resp_payload[3]) << 8);
    return true;
}

bool ServoBus::VinLimitWrite(uint8_t id, uint16_t min_mv, uint16_t max_mv)
{
    uint8_t params[4];
    params[0] = static_cast<uint8_t>(min_mv & 0xFF);
    params[1] = static_cast<uint8_t>((min_mv >> 8) & 0xFF);
    params[2] = static_cast<uint8_t>(max_mv & 0xFF);
    params[3] = static_cast<uint8_t>((max_mv >> 8) & 0xFF);
    return SendPacket(id, 22, params, 4) != 0;
}

bool ServoBus::VinLimitRead(uint8_t id, uint16_t &out_min_mv, uint16_t &out_max_mv, uint32_t timeout_ms)
{
    SendPacket(id, 23, nullptr, 0);
    if (!WaitForResponse(23, timeout_ms)) return false;
    if (s_resp_payload_len < 4) return false;
    out_min_mv = static_cast<uint16_t>(s_resp_payload[0]) | (static_cast<uint16_t>(s_resp_payload[1]) << 8);
    out_max_mv = static_cast<uint16_t>(s_resp_payload[2]) | (static_cast<uint16_t>(s_resp_payload[3]) << 8);
    return true;
}

bool ServoBus::TempMaxLimitWrite(uint8_t id, uint8_t temp_deg)
{
    uint8_t params[1] = { temp_deg };
    return SendPacket(id, 24, params, 1) != 0;
}

bool ServoBus::TempMaxLimitRead(uint8_t id, uint8_t &out_temp_deg, uint32_t timeout_ms)
{
    SendPacket(id, 25, nullptr, 0);
    if (!WaitForResponse(25, timeout_ms)) return false;
    if (s_resp_payload_len < 1) return false;
    out_temp_deg = s_resp_payload[0];
    return true;
}

bool ServoBus::TempRead(uint8_t id, uint8_t &out_temp_deg, uint32_t timeout_ms)
{
    SendPacket(id, 26, nullptr, 0);
    if (!WaitForResponse(26, timeout_ms)) return false;
    if (s_resp_payload_len < 1) return false;
    out_temp_deg = s_resp_payload[0];
    return true;
}

bool ServoBus::VinRead(uint8_t id, uint16_t &out_mv, uint32_t timeout_ms)
{
    SendPacket(id, 27, nullptr, 0);
    if (!WaitForResponse(27, timeout_ms)) return false;
    if (s_resp_payload_len < 2) return false;
    out_mv = static_cast<uint16_t>(s_resp_payload[0]) | (static_cast<uint16_t>(s_resp_payload[1]) << 8);
    return true;
}

bool ServoBus::PosRead(uint8_t id, int16_t &out_pos_raw, uint32_t timeout_ms)
{
    SendPacket(id, 28, nullptr, 0);
    if (!WaitForResponse(28, timeout_ms)) return false;
    if (s_resp_payload_len < 2) return false;
    out_pos_raw = static_cast<int16_t>(static_cast<uint16_t>(s_resp_payload[0]) | (static_cast<uint16_t>(s_resp_payload[1]) << 8));
    return true;
}

bool ServoBus::OrMotorModeRead(uint8_t id, uint8_t &out_mode, int16_t &out_speed, uint32_t timeout_ms)
{
    SendPacket(id, 30, nullptr, 0);
    if (!WaitForResponse(30, timeout_ms)) return false;
    if (s_resp_payload_len < 4) return false;
    out_mode = s_resp_payload[0];
    out_speed = static_cast<int16_t>(static_cast<uint16_t>(s_resp_payload[2]) | (static_cast<uint16_t>(s_resp_payload[3]) << 8));
    return true;
}

bool ServoBus::LoadOrUnloadWrite(uint8_t id, uint8_t load)
{
    uint8_t params[1] = { load };
    return SendPacket(id, 31, params, 1) != 0;
}

bool ServoBus::LoadOrUnloadRead(uint8_t id, uint8_t &out_load, uint32_t timeout_ms)
{
    SendPacket(id, 32, nullptr, 0);
    if (!WaitForResponse(32, timeout_ms)) return false;
    if (s_resp_payload_len < 1) return false;
    out_load = s_resp_payload[0];
    return true;
}

bool ServoBus::LedCtrlWrite(uint8_t id, uint8_t led_state)
{
    uint8_t params[1] = { led_state };
    return SendPacket(id, 33, params, 1) != 0;
}

bool ServoBus::LedCtrlRead(uint8_t id, uint8_t &out_led_state, uint32_t timeout_ms)
{
    SendPacket(id, 34, nullptr, 0);
    if (!WaitForResponse(34, timeout_ms)) return false;
    if (s_resp_payload_len < 1) return false;
    out_led_state = s_resp_payload[0];
    return true;
}

bool ServoBus::LedErrorWrite(uint8_t id, uint8_t mask)
{
    uint8_t params[1] = { mask };
    return SendPacket(id, 35, params, 1) != 0;
}

bool ServoBus::LedErrorRead(uint8_t id, uint8_t &out_mask, uint32_t timeout_ms)
{
    SendPacket(id, 36, nullptr, 0);
    if (!WaitForResponse(36, timeout_ms)) return false;
    if (s_resp_payload_len < 1) return false;
    out_mask = s_resp_payload[0];
    return true;
}

// UART receive callback used to parse servo responses
void ServoBus::ServoBus_UartCallback(uint8_t *buffer, uint16_t length)
{
    // scan for valid frames in buffer
    for (uint16_t i = 0; i + 5 < length; ++i) {
        if (buffer[i] == 0x55 && buffer[i+1] == 0x55) {
            uint8_t id = buffer[i+2];
            uint8_t len = buffer[i+3];
            uint8_t cmd = buffer[i+4];
            uint16_t frame_end = i + 4 + len; // index of last param (Cmd + params) -> checksum at +1
            if (frame_end >= length) continue; // not enough data
            uint8_t checksum = buffer[frame_end + 1];
            // compute checksum
            uint16_t sum = id + len + cmd;
            for (uint8_t p = 0; p < (len - 1); ++p) {
                sum += buffer[i + 5 + p];
            }
            uint8_t calc = static_cast<uint8_t>(~(sum & 0xFF));
            if (calc != checksum) continue;

            // extract payload
            uint8_t payload_len = (len >= 1) ? (len - 1) : 0;
            if (payload_len > sizeof(s_resp_payload)) payload_len = sizeof(s_resp_payload);
            for (uint8_t p = 0; p < payload_len; ++p) {
                s_resp_payload[p] = buffer[i + 5 + p];
            }
            s_resp_payload_len = payload_len;
            s_resp_cmd = cmd;
            s_resp_id = id;
            s_resp_ready = true;
            return;
        }
    }
}

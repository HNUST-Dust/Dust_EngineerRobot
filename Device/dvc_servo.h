#ifndef DVC_SERVO_H
#define DVC_SERVO_H

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"
#include <cstdint>
#include <array>

class ServoBus {
public:
    explicit ServoBus(UART_HandleTypeDef *huart = nullptr);

    void Init(UART_HandleTypeDef *huart);

    // Move to absolute angle in degrees (0..240). time_ms: move duration in ms.
    bool MoveToAngle(uint8_t id, float degrees, uint32_t time_ms);

    // Set motor mode speed (rpm-like unit used by user; protocol expects -1000..1000)
    bool SetSpeed(uint8_t id, int16_t speed);

    // Stop moving immediately
    bool Stop(uint8_t id);

    // Start previously set wait-move commands
    bool MoveStart(uint8_t id);

    // forward declaration for UART RX callback registration
    void ServoBus_UartCallback(uint8_t *buffer, uint16_t length);

    // Read servo ID. If `query_id` is 0xFE (broadcast) and exactly one servo responds,
    // this returns that servo's ID; otherwise returns 0xFF on timeout/failure.
    uint8_t ReadID(uint8_t query_id = 0xFE, uint32_t timeout_ms = 200);

    // Write servo ID: send to `current_id` to set `new_id`. Returns true if sent.
    bool WriteID(uint8_t current_id, uint8_t new_id);

    // Move with wait (set preset but do not start)
    bool MoveTimeWait(uint8_t id, float degrees, uint32_t time_ms);

    // Read current move target/time (Cmd 2) -> returns angle raw (0..1000) and time_ms
    bool ReadMoveTime(uint8_t id, uint16_t &out_angle_raw, uint32_t &out_time_ms, uint32_t timeout_ms = 200);

    // Read preset wait move (Cmd 8)
    bool ReadMoveTimeWait(uint8_t id, uint16_t &out_angle_raw, uint32_t &out_time_ms, uint32_t timeout_ms = 200);

    // Angle offset adjust/write/read (Cmd 17/18/19)
    bool AngleOffsetAdjust(uint8_t id, int8_t offset);
    bool AngleOffsetWrite(uint8_t id);
    bool AngleOffsetRead(uint8_t id, int8_t &out_offset, uint32_t timeout_ms = 200);

    // Angle limit write/read (Cmd 20/21)
    bool AngleLimitWrite(uint8_t id, uint16_t min_raw, uint16_t max_raw);
    bool AngleLimitRead(uint8_t id, uint16_t &out_min_raw, uint16_t &out_max_raw, uint32_t timeout_ms = 200);

    // VIN limit write/read (Cmd 22/23)
    bool VinLimitWrite(uint8_t id, uint16_t min_mv, uint16_t max_mv);
    bool VinLimitRead(uint8_t id, uint16_t &out_min_mv, uint16_t &out_max_mv, uint32_t timeout_ms = 200);

    // Temp max limit write/read (Cmd 24/25)
    bool TempMaxLimitWrite(uint8_t id, uint8_t temp_deg);
    bool TempMaxLimitRead(uint8_t id, uint8_t &out_temp_deg, uint32_t timeout_ms = 200);

    // Temp read (26) -> returns temperature deg C
    bool TempRead(uint8_t id, uint8_t &out_temp_deg, uint32_t timeout_ms = 200);

    // Vin read (27) -> returns mv
    bool VinRead(uint8_t id, uint16_t &out_mv, uint32_t timeout_ms = 200);

    // Position read (28) -> returns signed short raw position
    bool PosRead(uint8_t id, int16_t &out_pos_raw, uint32_t timeout_ms = 200);

    // Read mode/speed (Cmd 30)
    bool OrMotorModeRead(uint8_t id, uint8_t &out_mode, int16_t &out_speed, uint32_t timeout_ms = 200);

    // Load/unload write/read (31/32)
    bool LoadOrUnloadWrite(uint8_t id, uint8_t load);
    bool LoadOrUnloadRead(uint8_t id, uint8_t &out_load, uint32_t timeout_ms = 200);

    // LED ctrl write/read (33/34)
    bool LedCtrlWrite(uint8_t id, uint8_t led_state);
    bool LedCtrlRead(uint8_t id, uint8_t &out_led_state, uint32_t timeout_ms = 200);

    // LED error write/read (35/36)
    bool LedErrorWrite(uint8_t id, uint8_t mask);
    bool LedErrorRead(uint8_t id, uint8_t &out_mask, uint32_t timeout_ms = 200);

private:
    UART_HandleTypeDef *huart_;

    uint8_t SendPacket(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t params_len);
    static uint16_t AngleToRaw(float degrees);
};

#endif // DVC_SERVO_H

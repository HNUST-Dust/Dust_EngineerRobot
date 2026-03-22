#include "pid_tuner.h"

#include "../Algorithm/control/alg_pid.h"

#include "FreeRTOS.h"
#include "queue.h"

#include <cctype>
#include <cstring>

PidTuner g_pid_tuner;

namespace {
constexpr UBaseType_t kQueueDepth = 32;
}

void PidTuner::Init()
{
    if (queue_ != nullptr) {
        return;
    }

    QueueHandle_t q = xQueueCreate(kQueueDepth, sizeof(PidTunerMsg));
    queue_ = q;
    accum_len_ = 0;
}

bool PidTuner::RegisterTarget(const char* name, alg::Pid* pid)
{
    if (name == nullptr || pid == nullptr) {
        return false;
    }
    if (target_count_ >= kMaxTargets) {
        return false;
    }

    targets_[target_count_].name = name;
    targets_[target_count_].pid = pid;
    ++target_count_;
    if (target_count_ == 1) {
        selected_target_ = 0;
    }
    return true;
}

bool PidTuner::Pop(PidTunerMsg& out)
{
    if (queue_ == nullptr) {
        return false;
    }

    return xQueueReceive(static_cast<QueueHandle_t>(queue_), &out, 0) == pdTRUE;
}

namespace {

void ApplyPidField(alg::Pid& pid, uint8_t field_id, float value)
{
    switch (field_id) {
        case 0x01: pid.SetKp(value); break;
        case 0x02: pid.SetKi(value); break;
        case 0x03: pid.SetKd(value); break;
        case 0x04: pid.SetKf(value); break;
        case 0x05: pid.SetIOutMax(value); break;
        case 0x06: pid.SetOutMax(value); break;
        default: break;
    }
}

} // namespace

void PidTuner::ProcessInTask()
{
    PidTunerMsg msg;
    while (Pop(msg)) {
        if (msg.cmd == kCmdSelect) {
            if (msg.arg < target_count_) {
                selected_target_ = msg.arg;
            }
            continue;
        }

        if (target_count_ == 0 || selected_target_ >= target_count_) {
            continue;
        }
        alg::Pid* pid = targets_[selected_target_].pid;
        if (pid == nullptr) {
            continue;
        }

        if (msg.cmd == kCmdSet) {
            ApplyPidField(*pid, msg.arg, msg.value);
            if ((msg.flags & kFlagResetAfterSet) != 0) {
                pid->reset();
            }
        } else if (msg.cmd == kCmdReset) {
            pid->reset();
        }
    }
}

void PidTuner::OnRx(const uint8_t* data, uint16_t len)
{
    if (queue_ == nullptr || data == nullptr || len == 0) {
        return;
    }

    // Append to accum_ (drop oldest if overflow).
    if (len >= kAccumCap) {
        // Keep only the last kAccumCap bytes.
        std::memcpy(accum_, data + (len - kAccumCap), kAccumCap);
        accum_len_ = kAccumCap;
    } else {
        const uint16_t free_space = static_cast<uint16_t>(kAccumCap - accum_len_);
        if (len > free_space) {
            const uint16_t drop = static_cast<uint16_t>(len - free_space);
            if (drop >= accum_len_) {
                accum_len_ = 0;
            } else {
                std::memmove(accum_, accum_ + drop, static_cast<size_t>(accum_len_ - drop));
                accum_len_ = static_cast<uint16_t>(accum_len_ - drop);
            }
        }

        std::memcpy(accum_ + accum_len_, data, len);
        accum_len_ = static_cast<uint16_t>(accum_len_ + len);
    }

    TryParseLinesFromAccum();
}

namespace {

inline bool IsSpace(uint8_t c)
{
    return std::isspace(static_cast<unsigned char>(c)) != 0;
}

inline void TrimSpan(const uint8_t*& begin, const uint8_t*& end)
{
    while (begin < end && IsSpace(*begin)) {
        ++begin;
    }
    while (end > begin && IsSpace(*(end - 1))) {
        --end;
    }
}

inline bool SpanEqualsIgnoreCase(const uint8_t* begin, const uint8_t* end, const char* lit)
{
    const size_t n = static_cast<size_t>(end - begin);
    size_t i = 0;
    for (; lit[i] != '\0'; ++i) {
        if (i >= n) {
            return false;
        }
        const uint8_t a = begin[i];
        const uint8_t b = static_cast<uint8_t>(lit[i]);
        const uint8_t al = (a >= 'A' && a <= 'Z') ? static_cast<uint8_t>(a + ('a' - 'A')) : a;
        const uint8_t bl = (b >= 'A' && b <= 'Z') ? static_cast<uint8_t>(b + ('a' - 'A')) : b;
        if (al != bl) {
            return false;
        }
    }
    return i == n;
}

inline bool ParseUint(const uint8_t* begin, const uint8_t* end, uint32_t& out)
{
    TrimSpan(begin, end);
    if (begin >= end) {
        return false;
    }

    int base = 10;
    if ((end - begin) >= 2 && begin[0] == '0' && (begin[1] == 'x' || begin[1] == 'X')) {
        base = 16;
        begin += 2;
    }

    uint32_t v = 0;
    bool any = false;
    while (begin < end) {
        const uint8_t c = *begin++;
        uint32_t d;
        if (c >= '0' && c <= '9') {
            d = static_cast<uint32_t>(c - '0');
        } else if (base == 16 && c >= 'a' && c <= 'f') {
            d = static_cast<uint32_t>(10 + (c - 'a'));
        } else if (base == 16 && c >= 'A' && c <= 'F') {
            d = static_cast<uint32_t>(10 + (c - 'A'));
        } else if (IsSpace(c)) {
            // allow trailing spaces
            while (begin < end && IsSpace(*begin)) {
                ++begin;
            }
            break;
        } else {
            return false;
        }
        if (d >= static_cast<uint32_t>(base)) {
            return false;
        }
        v = v * static_cast<uint32_t>(base) + d;
        any = true;
    }

    if (!any) {
        return false;
    }
    out = v;
    return true;
}

inline bool ParseFloatSimple(const uint8_t* begin, const uint8_t* end, float& out)
{
    // Fixed-precision parser, quantized to 5 decimal places (1e-5).
    // Accept: [-]?[0-9]+(.[0-9]+)? with 0..N fractional digits.
    // If more than 5 fractional digits are provided, round to 5 digits.
    TrimSpan(begin, end);
    if (begin >= end) {
        return false;
    }

    bool neg = false;
    if (*begin == '+' || *begin == '-') {
        neg = (*begin == '-');
        ++begin;
    }
    if (begin >= end) {
        return false;
    }

    uint32_t int_part = 0;
    bool any_int = false;
    while (begin < end && *begin >= '0' && *begin <= '9') {
        any_int = true;
        int_part = int_part * 10u + static_cast<uint32_t>(*begin - '0');
        ++begin;
    }
    if (!any_int) {
        return false;
    }

    uint32_t frac_part_5 = 0;
    uint8_t frac_digits = 0;
    uint8_t rounding_digit = 0;
    bool has_more_frac_digits = false;

    if (begin < end && *begin == '.') {
        ++begin;
        while (begin < end && *begin >= '0' && *begin <= '9') {
            const uint8_t d = static_cast<uint8_t>(*begin - '0');
            if (frac_digits < 5) {
                frac_part_5 = frac_part_5 * 10u + static_cast<uint32_t>(d);
                ++frac_digits;
            } else if (frac_digits == 5) {
                rounding_digit = d;
                ++frac_digits;
            } else {
                has_more_frac_digits = true;
            }
            ++begin;
        }
    }

    // Allow trailing whitespace only.
    while (begin < end) {
        if (!IsSpace(*begin)) {
            return false;
        }
        ++begin;
    }

    // Scale fractional part to 5 digits.
    if (frac_digits == 0) {
        frac_part_5 = 0;
    } else if (frac_digits > 0 && frac_digits < 5) {
        // e.g. 1.23 -> 23000
        for (; frac_digits < 5; ++frac_digits) {
            frac_part_5 *= 10u;
        }
    } else if (frac_digits >= 6) {
        // We saw at least one digit beyond 5; round based on the 6th digit.
        (void)has_more_frac_digits; // currently unused (could be used for tie-breaking)
        if (rounding_digit >= 5u) {
            ++frac_part_5;
            if (frac_part_5 >= 100000u) {
                frac_part_5 = 0;
                ++int_part;
            }
        }
    }

    const int32_t scaled = static_cast<int32_t>(int_part) * 100000 + static_cast<int32_t>(frac_part_5);
    out = (neg ? -1.0f : 1.0f) * (static_cast<float>(scaled) * 0.00001f);
    return true;
}

} // namespace

void PidTuner::TryParseLinesFromAccum()
{
    // Parse complete lines ended by '\n'. Accept CRLF (strip '\r').
    while (accum_len_ > 0) {
        uint16_t nl = 0;
        bool found = false;
        for (; nl < accum_len_; ++nl) {
            if (accum_[nl] == '\n') {
                found = true;
                break;
            }
        }
        if (!found) {
            return;
        }

        // Define [line_begin, line_end) without the trailing '\n' and optional '\r'.
        const uint8_t* line_begin = accum_;
        const uint8_t* line_end = accum_ + nl;
        if (line_end > line_begin && *(line_end - 1) == '\r') {
            --line_end;
        }
        TrimSpan(line_begin, line_end);

        if (line_begin < line_end) {
            // Split at ':'
            const uint8_t* colon = nullptr;
            for (const uint8_t* p = line_begin; p < line_end; ++p) {
                if (*p == ':') {
                    colon = p;
                    break;
                }
            }

            const uint8_t* key_begin = line_begin;
            const uint8_t* key_end = colon ? colon : line_end;
            const uint8_t* val_begin = colon ? (colon + 1) : line_end;
            const uint8_t* val_end = line_end;
            TrimSpan(key_begin, key_end);
            TrimSpan(val_begin, val_end);

            // pid:<name|slot> selects target pid (enqueued as kCmdSelect).
            if (SpanEqualsIgnoreCase(key_begin, key_end, "pid")) {
                if (target_count_ == 0) {
                    // No targets registered yet.
                } else {
                    uint8_t selected_index = 0xFF;
                    uint32_t slot_u32 = 0;
                    if (ParseUint(val_begin, val_end, slot_u32)) {
                        // Slot is 1-based by convention. Also accept 0-based.
                        if (slot_u32 < target_count_) {
                            selected_index = static_cast<uint8_t>(slot_u32);
                        } else if (slot_u32 >= 1 && (slot_u32 - 1) < target_count_) {
                            selected_index = static_cast<uint8_t>(slot_u32 - 1);
                        }
                    } else {
                        for (uint8_t i = 0; i < target_count_; ++i) {
                            const char* name = targets_[i].name;
                            if (name != nullptr && SpanEqualsIgnoreCase(val_begin, val_end, name)) {
                                selected_index = i;
                                break;
                            }
                        }
                    }

                    if (selected_index != 0xFF) {
                        PidTunerMsg msg;
                        msg.cmd = kCmdSelect;
                        msg.flags = 0;
                        msg.arg = selected_index;
                        msg.reserved = 0;
                        msg.value = 0.0f;
                        msg.seq = static_cast<uint16_t>(++seq_);

                        BaseType_t higher_priority_woken = pdFALSE;
                        (void)xQueueSendFromISR(static_cast<QueueHandle_t>(queue_), &msg, &higher_priority_woken);
                        portYIELD_FROM_ISR(higher_priority_woken);
                    }
                }
            } else {
                // Map key -> field id / command
                uint8_t cmd = 0;
                uint8_t field_id = 0;
                float value_f = 0.0f;
                bool has_value = false;
                bool reset_after_set = false;

                if (SpanEqualsIgnoreCase(key_begin, key_end, "reset") || SpanEqualsIgnoreCase(key_begin, key_end, "rst")) {
                    uint32_t v = 0;
                    if (ParseUint(val_begin, val_end, v) && v != 0) {
                        cmd = kCmdReset;
                    }
                } else {
                    cmd = kCmdSet;
                    if (SpanEqualsIgnoreCase(key_begin, key_end, "kp")) {
                        field_id = 0x01;
                    } else if (SpanEqualsIgnoreCase(key_begin, key_end, "ki")) {
                        field_id = 0x02;
                    } else if (SpanEqualsIgnoreCase(key_begin, key_end, "kd")) {
                        field_id = 0x03;
                    } else if (SpanEqualsIgnoreCase(key_begin, key_end, "kf")) {
                        field_id = 0x04;
                    } else if (SpanEqualsIgnoreCase(key_begin, key_end, "i_out_max") || SpanEqualsIgnoreCase(key_begin, key_end, "ioutmax")) {
                        field_id = 0x05;
                    } else if (SpanEqualsIgnoreCase(key_begin, key_end, "out_max") || SpanEqualsIgnoreCase(key_begin, key_end, "outmax")) {
                        field_id = 0x06;
                    } else if (SpanEqualsIgnoreCase(key_begin, key_end, "kp!") || SpanEqualsIgnoreCase(key_begin, key_end, "ki!") ||
                               SpanEqualsIgnoreCase(key_begin, key_end, "kd!") || SpanEqualsIgnoreCase(key_begin, key_end, "kf!")) {
                        // Allow "kp!:1" meaning set + reset.
                        reset_after_set = true;
                        // Strip '!'
                        const uint8_t* k2_end = key_end - 1;
                        if (SpanEqualsIgnoreCase(key_begin, k2_end, "kp")) field_id = 0x01;
                        else if (SpanEqualsIgnoreCase(key_begin, k2_end, "ki")) field_id = 0x02;
                        else if (SpanEqualsIgnoreCase(key_begin, k2_end, "kd")) field_id = 0x03;
                        else if (SpanEqualsIgnoreCase(key_begin, k2_end, "kf")) field_id = 0x04;
                    } else {
                        cmd = 0;
                    }

                    if (cmd == kCmdSet && field_id != 0) {
                        has_value = ParseFloatSimple(val_begin, val_end, value_f);
                        if (!has_value) {
                            cmd = 0;
                        }
                    }
                }

                if (cmd != 0) {
                    PidTunerMsg msg;
                    msg.cmd = cmd;
                    msg.flags = 0;
                    if (reset_after_set) {
                        msg.flags |= kFlagResetAfterSet;
                    }
                    msg.arg = field_id;
                    msg.reserved = 0;
                    msg.value = value_f;
                    msg.seq = static_cast<uint16_t>(++seq_);

                    BaseType_t higher_priority_woken = pdFALSE;
                    (void)xQueueSendFromISR(static_cast<QueueHandle_t>(queue_), &msg, &higher_priority_woken);
                    portYIELD_FROM_ISR(higher_priority_woken);
                }
            }
        }

        // Consume this line (including '\n').
        const uint16_t consume = static_cast<uint16_t>(nl + 1);
        if (consume >= accum_len_) {
            accum_len_ = 0;
            return;
        }
        std::memmove(accum_, accum_ + consume, static_cast<size_t>(accum_len_ - consume));
        accum_len_ = static_cast<uint16_t>(accum_len_ - consume);
    }
}

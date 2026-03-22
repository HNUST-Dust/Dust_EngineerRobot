# 幻尔科技总线舵机通信协议

## 概要

采用异步串行总线通讯方式，最多可串联 253 个舵机（ID 0～253），通过 UART 异步串行接口统一控制。每个舵机可设置不同节点地址，支持整体或单个控制。支持两种模式：电机控制模式（速度控制）和位置控制模式（0–240°，可加 ±30° 偏差）。只要符合半双工 UART 串口、波特率 115200bps 的接口即可通讯。

## UART 接口原理图

舵机通过程序控制 UART 时序实现半双工总线通讯（115200bps）。控制器端需按照硬件原理实现半双工收发（参见原厂电路图）。

## 指令包格式

帧结构（从前到后）：

| 帧头 (2B) | ID (1B) | Length (1B) | Cmd (1B) | Params (N B) | Checksum (1B) |
|---:|---:|---:|---:|---:|---:|
| 0x55 0x55 | ID | Length | Cmd | Prm1 ... PrmN | Checksum |

字段说明：
- 帧头：连续两个 `0x55` 表示包起始。
- ID：舵机 ID，范围 0–253（0x00–0xFD）。广播 ID 为 254（0xFE），广播包所有舵机收到但通常不返回应答（读取 ID 除外，见下文）。
- Length：数据长度，等于本数据段（包含 Length 本身）长度；从 Length 到 Checksum 的字节总数为 Length+3（含 ID、Length、Cmd 等）。
- Cmd：指令码。
- Params：指令参数（可为空）。
- Checksum：校验和，计算方法：

Checksum = ~ (ID + Length + Cmd + Prm1 + ... + PrmN) & 0xFF

（先对括号内求和取最低字节，再按位取反）

## 指令类型说明

指令分两类：
- 写指令（*_WRITE）：携带参数，写入设置或触发动作，舵机接收后执行（有些为延迟执行）。
- 读指令（*_READ）：通常无参数，舵机收到后会立即返回对应带参数的响应包，响应的 Cmd 与读指令一致。发送读指令后上位机需立刻切换为接收状态以读取返回数据。

下面列出常用指令，按指令值排序。参数描述均以字节为单位，必要时注明范围及单位。

1. **SERVO_MOVE_TIME_WRITE** — Cmd: `1`，Length: `7`
   - 参数 1：目标角度低八位（0–1000，对应 0–240°，分辨率 0.24°）
   - 参数 2：目标角度高八位
   - 参数 3：移动时间低八位（ms）
   - 参数 4：移动时间高八位（ms，范围 0–30000）
   - 说明：舵机在指定时间内匀速从当前位置移动到目标角度，收到后立即开始动作。

2. **SERVO_MOVE_TIME_READ** — Cmd: `2`，Length: `3`
   - 说明：读取上面写指令的目标角度和时间，舵机会以带参数的响应包返回相同 Cmd。

3. **SERVO_MOVE_TIME_WAIT_WRITE** — Cmd: `7`，Length: `7`
   - 参数 1/2：预设角度（低/高 八位，0–1000）
   - 参数 3/4：预设时间（低/高 八位，ms，0–30000）
   - 说明：与 SERVO_MOVE_TIME_WRITE 类似，但收到后不立即执行，须待收到 `SERVO_MOVE_START`（Cmd `11`）后才开始动作。

4. **SERVO_MOVE_TIME_WAIT_READ** — Cmd: `8`，Length: `3`
   - 说明：读取预设角度与时间。

5. **SERVO_MOVE_START** — Cmd: `11`，Length: `3`
   - 说明：触发之前由 `SERVO_MOVE_TIME_WAIT_WRITE` 设置的动作开始执行。

6. **SERVO_MOVE_STOP** — Cmd: `12`，Length: `3`
   - 说明：若舵机正在转动，立即停止并保持当前位置。

7. **SERVO_ID_WRITE** — Cmd: `13`，Length: `4`
   - 参数 1：新的舵机 ID（0–253），写入并掉电保存。

8. **SERVO_ID_READ** — Cmd: `14`，Length: `3`
   - 说明：读取舵机 ID。注意：此指令在使用广播 ID (`0xFE`) 时舵机会返回应答，便于未知 ID 时通过广播查询（总线上仅接一台舵机时可用，避免冲突）。

9. **SERVO_ANGLE_OFFSET_ADJUST** — Cmd: `17`，Length: `4`
   - 参数 1：偏差值（signed char，范围 -125～125，对应 -30°～30°）
   - 说明：到达后舵机会立即按该偏差转动。此指令不支持掉电保存（若需保存请用下一条写入）。
   - 注意：参数为有符号类型，发送前需按协议转换为 unsigned char。

10. **SERVO_ANGLE_OFFSET_WRITE** — Cmd: `18`，Length: `3`
    - 说明：保存当前偏差值（支持掉电保存）。

11. **SERVO_ANGLE_OFFSET_READ** — Cmd: `19`，Length: `3`
    - 说明：读取保存的偏差值（signed，-125～125）。

12. **SERVO_ANGLE_LIMIT_WRITE** — Cmd: `20`，Length: `7`
    - 参数 1/2：最小角度（低/高 八位，0–1000）
    - 参数 3/4：最大角度（低/高 八位，0–1000），要求最小 < 最大，写入并支持掉电保存。
    - 说明：限制舵机活动范围在最小到最大之间。

13. **SERVO_ANGLE_LIMIT_READ** — Cmd: `21`，Length: `3`
    - 说明：读取角度限制值。

14. **SERVO_VIN_LIMIT_WRITE** — Cmd: `22`，Length: `7`
    - 参数 1/2：最小输入电压（低/高 八位，单位 mV，范围 4500–12000）
    - 参数 3/4：最大输入电压（低/高 八位，单位 mV，范围 4500–12000），要求最小 < 最大，支持掉电保存。
    - 说明：超出范围时（若开启 LED 报警）会闪烁并卸载电机断电以保护舵机。

15. **SERVO_VIN_LIMIT_READ** — Cmd: `23`，Length: `3`
    - 说明：读取电压限制值。

16. **SERVO_TEMP_MAX_LIMIT_WRITE** — Cmd: `24`，Length: `4`
    - 参数 1：最高温度限制（℃，范围 50–100，默认 85），超过则触发（可选）LED 报警并卸载电机断电，支持掉电保存。

17. **SERVO_TEMP_MAX_LIMIT_READ** — Cmd: `25`，Length: `3`
    - 说明：读取温度限制值。

18. **SERVO_TEMP_READ** — Cmd: `26`，Length: `3`
    - 说明：读取舵机当前温度。

19. **SERVO_VIN_READ** — Cmd: `27`，Length: `3`
    - 说明：读取舵机当前输入电压（返回 2 字节，低八位/高八位）。

20. **SERVO_POS_READ** — Cmd: `28`，Length: `3`
    - 说明：读取舵机当前位置（返回 2 字节，低八位/高八位）。返回值需转换为 signed short（可能为负）。

21. **SERVO_OR_MOTOR_MODE_WRITE** — Cmd: `29`，Length: `7`
    - 参数 1：模式（0=位置控制，1=电机控制，默认 0）
    - 参数 2：保留，写 0
    - 参数 3/4：速度（signed short，低/高 八位，范围 -1000～1000，仅电机模式有效）
    - 注意：速度为有符号短整型，发送前转换为 unsigned short 传输；模式与速度不支持掉电保存。

22. **SERVO_OR_MOTOR_MODE_READ** — Cmd: `30`，Length: `3`
    - 说明：读取当前模式与速度参数。

23. **SERVO_LOAD_OR_UNLOAD_WRITE** — Cmd: `31`，Length: `4`
    - 参数 1：是否装载电机（0=卸载/无力矩，1=装载/有力矩，默认 0）

24. **SERVO_LOAD_OR_UNLOAD_READ** — Cmd: `32`，Length: `3`
    - 说明：读取电机装载状态。

25. **SERVO_LED_CTRL_WRITE** — Cmd: `33`，Length: `4`
    - 参数 1：LED 状态（0=常亮，1=常灭，默认 0），支持掉电保存。

26. **SERVO_LED_CTRL_READ** — Cmd: `34`，Length: `3`
    - 说明：读取 LED 状态。

27. **SERVO_LED_ERROR_WRITE** — Cmd: `35`，Length: `4`
    - 参数 1：故障触发 LED 报警的掩码（0–7），位含义：
      - 0：无报警
      - 1：过温
      - 2：过压
      - 3：过温 + 过压
      - 4：堵转
      - 5：过温 + 堵转
      - 6：过压 + 堵转
      - 7：过温 + 过压 + 堵转

28. **SERVO_LED_ERROR_READ** — Cmd: `36`，Length: `3`
    - 说明：读取故障掩码。

## 舵机响应（READ 指令的返回包）

响应包的格式与发送包相同（见“指令包格式”），Cmd 值与读指令一致，且携带相应参数：
- `SERVO_MOVE_TIME_READ`（Cmd `2`，Length `7`）：返回目标角度（低/高）与时间（低/高）。
- `SERVO_MOVE_TIME_WAIT_READ`（Cmd `8`，Length `7`）：返回预设角度与时间。
- `SERVO_ID_READ`（Cmd `14`，Length `4`）：返回当前 ID（参数 1）。广播查询允许单舵机返回。
- `SERVO_ANGLE_OFFSET_READ`（Cmd `19`，Length `4`）：返回偏差值（signed）。
- `SERVO_ANGLE_LIMIT_READ`（Cmd `21`，Length `7`）：返回最小/最大角度（各低/高 八位）。
- `SERVO_VIN_LIMIT_READ`（Cmd `23`，Length `7`）：返回最小/最大输入电压（mV）。
- `SERVO_TEMP_MAX_LIMIT_READ`（Cmd `25`，Length `4`）：返回温度上限。
- `SERVO_TEMP_READ`（Cmd `26`，Length `4`）：返回当前温度。
- `SERVO_VIN_READ`（Cmd `27`，Length `5`）：返回输入电压低/高 八位。
- `SERVO_POS_READ`（Cmd `28`，Length `5`）：返回当前位置低/高 八位（需转 signed short）。
- `SERVO_OR_MOTOR_MODE_READ`（Cmd `30`，Length `7`）：返回模式、保留、速度低/高。
- `SERVO_LOAD_OR_UNLOAD_READ`（Cmd `32`，Length `4`）：返回装载状态。
- `SERVO_LED_CTRL_READ`（Cmd `34`，Length `4`）：返回 LED 状态。
- `SERVO_LED_ERROR_READ`（Cmd `36`，Length `4`）：返回故障掩码。

## 备注

- 所有多字节数值均采用低字节在前（Little Endian）。
- 发送有符号类型时（如 speed、offset），请在发送前转换为对应的无符号表示以便在传输中不丢失位表示，接收端再转换回有符号类型。
- 广播 ID (`0xFE`) 的读指令一般不返回应答，唯一例外为 `SERVO_ID_READ`。

# 上位机与下位机通信协议

本文是上位机 ROS2 与 STM32 下位机之间 USB 线协议的唯一规范源。上下位机实现文档只引用本协议，不得另行定义不同帧格式。

## 1. 实现阶段

| 阶段 | 必须实现 | 可暂不实现 |
| --- | --- | --- |
| 第一阶段：任务赛通讯 | `0x10 VIRTUAL_RC_SETPOINT`，50 Hz | `0x11` IMU、下位机状态回传、软件急停帧 |
| 第二阶段第一步：无 IMU 障碍动作 | 继续使用 `0x10`；`0x11` 合法帧可安全忽略 | IMU 数据接入、姿态超限停止和姿态补偿 |
| 第二阶段第二步：IMU 姿态增强 | `0x11 VIRTUAL_IMU`，25 Hz；新鲜度、方向和有限值校验 | 姿态补偿仅在超限停止验证后分步启用 |
| 后续协议扩展 | 状态回传、正式控制权握手、软件急停请求/确认 | 不得影响 v1 的 `0x10/0x11` 解码 |

第一阶段上位机可以只发送 `0x10`。如果发送端已经按 25 Hz 插入 `0x11`，尚未启用 IMU 的下位机也必须完整消费合法帧并安全忽略 payload。

## 2. 通道总览

| 通道 | 物理层 | 方向 | 用途 |
| --- | --- | --- | --- |
| 四足运动控制 | USB CDC | PC -> STM32 | 模式、运动轴、机械臂 jog |
| IMU 姿态 | USB CDC | PC -> STM32 | 第二阶段第二步姿态数据 |
| 机械臂动作 | UART TTL，115200 8N1 | PC -> 机械臂控制器 | HOME、抓取、放置和吸盘 |
| ROS2 内部 | DDS topics/services/actions | 节点间 | 导航、视觉、任务和安全协调 |

上位机打开 USB CDC 时配置 `115200 8N1`。USB CDC 本身不依赖物理串口波特率，但上下位机工具统一使用该配置。

## 3. USB 固定帧

`0x10` 与 `0x11` 均使用固定 40 字节包装：

```text
offset  size  field
0       1     magic0 = A5
1       1     magic1 = 5A
2       1     version = 01
3       1     msg_type = 10 / 11
4       2     reserved = 00 00
6       2     frame_seq，uint16 LE
8       2     payload_len = 1C 00
10      28    payload
38      2     crc16，uint16 LE
```

字节 2 的 `01` 是协议版本，字节 3 的 `10/11` 是消息类型，不能把 `01 10` 当作一个 16 位消息号。

### 3.1 CRC-16/CCITT-FALSE

```text
poly   = 0x1021
init   = 0xFFFF
refin  = false
refout = false
xorout = 0x0000
```

CRC 计算范围从字节 2 的 `version` 开始，到字节 37 的 payload 末尾结束，共 36 字节；不包含 `A5 5A` magic，也不包含末尾 CRC。CRC 在线上按低字节在前发送。按此范围计算，第 4.5 节测试向量得到 `0x96CF`。

```python
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc
```

### 3.2 字节流解析

接收端必须支持拆包、粘包、噪声跳过和错误后重同步：

1. 扫描 `A5 5A`。
2. 收满 40 字节。
3. 检查 version、msg_type、reserved、payload_len 和 CRC。
4. 合法帧按消息类型分发。
5. 错误时滑动一个字节继续寻找 magic，不清空整个 RX 缓冲。

## 4. `0x10 VIRTUAL_RC_SETPOINT`

第一阶段唯一必需的业务消息，固定 50 Hz 发送。Payload 等价于 Python：

```python
struct.Struct("<IIBBHhhhhhHI")
```

| payload 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | `uint32` | session_id | 上位机启动控制流时生成的随机非零 ID |
| 4 | `uint32` | host_time_ms | 上位机单调时钟低 32 位，仅诊断 |
| 8 | `uint8` | main_switch | 0=LOW，1=MID，2=HIGH |
| 9 | `uint8` | sub_switch | 0=LOW，1=MID，2=HIGH |
| 10 | `uint16` | command_flags | 位定义见 4.3 |
| 12 | `int16` | yaw_permille | CH1 等价量，`-1000..1000`，负左正右 |
| 14 | `int16` | forward_permille | CH2 等价量，`-1000..1000`，负退正进 |
| 16 | `int16` | speed_permille | CH3 等价量，`-1000..1000` |
| 18 | `int16` | arm_j0_permille | CH6 等价量，`-1000..1000` |
| 20 | `int16` | arm_j1_permille | CH7 等价量，`-1000..1000` |
| 22 | `uint16` | channel_valid_mask | v1 固定 `0x0067` |
| 24 | `uint32` | command_counter | session 内递增，允许 32 位回绕 |

### 4.1 第一阶段 session 规则

v1 第一阶段没有请求/响应握手。session_id 是控制流标识，不代表下位机已经回传授权：

1. 上位机每次进程启动或重新建立 USB 控制流时生成随机非零 session_id。
2. 前 3 帧必须是第 4.4 节的安全零帧。
3. 下位机只在实体遥控处于授权状态且收到安全零帧时锁定该 session。
4. 后续 `0x10` 必须保持相同 session_id，command_counter 必须前进。
5. USB 超时、拔线、实体接管或安全锁存后，下位机清除 session。
6. 重新控制必须使用新的 session_id 并重新发送 3 帧安全零帧。

正式双向 ACQUIRE/RELEASE 握手属于后续协议版本，不在第一阶段伪造实现。

### 4.2 九宫格模式

```text
virtual_mode = main_switch * 3 + sub_switch
```

| 编号 | 组合 | 名称 | 行为 |
| ---: | --- | --- | --- |
| 0 | LOW+LOW | IDLE | USB 安全待机，运动轴为零，轮驱 OFF/HOLD |
| 1 | LOW+MID | LOW_WHEEL | 低姿态轮行 |
| 2 | LOW+HIGH | SAFE_HOLD | 腿 RX-only，轮驱 HOLD |
| 3 | MID+LOW | STAND_HOLD | MIT 站立，轮驱 HOLD |
| 4 | MID+MID | STAND_WHEEL | 站立轮行 |
| 5 | MID+HIGH | STAND_ARM | 站立并允许 CH6/CH7 jog |
| 6 | HIGH+LOW | GAIT_ONLY | 纯足步态，轮驱 HOLD |
| 7 | HIGH+MID | GAIT_WHEEL | 步态与轮驱同步 |
| 8 | HIGH+HIGH | RESERVED | 保留，站立 HOLD |

USB 模式 0 是安全 IDLE，不等于当前实体遥控 LOW+LOW 的 MOTOR_CHECK/恢复动作。下位机必须使用独立 USB 映射。

### 4.3 command_flags

| 位 | 名称 | 行为 |
| ---: | --- | --- |
| 0 | `DEADMAN_HELD` | 未置位时 CH1/CH2/CH6/CH7 强制为零 |
| 1 | `MOTION_ENABLE` | 未置位时强制 IDLE、连续轴归零、CH3=-1000 |
| 2 | `SMOOTH_STOP` | CH1/CH2/CH6/CH7 强制为零并请求平稳收步 |

只有 `DEADMAN_HELD` 与 `MOTION_ENABLE` 同时置位时，才允许采用非零连续运动轴。未知 flag 位必须按无运动帧处理。

`channel_valid_mask=0x0067` 对应 CH1/CH2/CH3/CH6/CH7。未置位通道必须按 0 处理。USB payload 不存在 CH9。

### 4.4 安全零帧

```text
main_switch=0
sub_switch=0
flags=0
yaw=0
forward=0
speed=-1000
arm_j0=0
arm_j1=0
channel_valid_mask=0x0067
```

正常启动控制流、模式切换前、暂停和退出时至少发送 3 帧安全零帧。下位机还必须实现独立的 150 ms `0x10` 看门狗；不能依赖上位机一定发送停止帧。

### 4.5 联调测试向量

```text
session=0x11223344, host=0x01020304
HIGH+LOW, flags=0x0003
CH1=-250, CH2=500, CH3=-800, CH6/7=0
mask=0x0067, counter=10, frame_seq=0x1234

A5 5A 01 10 00 00 34 12 1C 00 44 33 22 11 04 03 02 01
02 00 03 00 06 FF F4 01 E0 FC 00 00 00 00 67 00 0A 00
00 00 CF 96
```

## 5. `0x11 VIRTUAL_IMU`

`0x11` 属于第二阶段第二步，25 Hz 发送。第一阶段和第二阶段第一步的下位机可以校验整帧后丢弃 payload。

Payload 等价于：

```python
struct.Struct("<6fI")
```

| payload 偏移 | 类型 | 字段 | 单位 |
| ---: | --- | --- | --- |
| 0 | `float32` | roll | deg |
| 4 | `float32` | pitch | deg |
| 8 | `float32` | yaw | deg |
| 12 | `float32` | gyro_x | rad/s |
| 16 | `float32` | gyro_y | rad/s |
| 20 | `float32` | gyro_z | rad/s |
| 24 | `uint32` | timestamp_ms | 上位机单调时钟低 32 位 |

第二阶段第二步接入时必须检查 6 个 float 均为有限值，将 rad/s 转成当前 `Dog_Imu_Sample.gyro_dps[]` 所需的 deg/s，并以本地接收时间判断新鲜度。首轮实机验证只用于记录、观察和姿态超限停止，不直接修改足端目标。

## 6. 控制与安全约束

| 约束 | v1 行为 |
| --- | --- |
| 实体优先 | 实体 SBUS 和 CH9 始终高于 USB |
| CH9 | USB 不能模拟、释放或清除 CH9 |
| USB 许可 | 实体遥控新鲜、CH9 释放、实体 LOW+LOW 且摇杆回中 |
| 超时 | 合法 `0x10` 超过 150 ms 未更新，轴归零、收步、轮驱 HOLD、清 session |
| 模式切换 | 先发 3 帧安全零帧，再持续发送新模式零轴帧，最后开放非零轴 |
| 无状态回传 | 第一阶段 UI 只能显示“命令已发送”，不得显示“下位机已确认” |
| 软件急停 | v1 未定义软件 ESTOP 帧；使用安全零帧和看门狗停车，危险情况使用实体 CH9/物理断电 |

状态回传和软件急停请求/确认必须在后续版本明确 msg_type、payload、重发和锁存语义后再实现。

### 6.1 ASCII 只读诊断兼容

第一阶段正式接口保留两个单字节只读诊断命令：

| 命令 | 返回内容 | 限制 |
| --- | --- | --- |
| `p` | 整机、电机、轮驱和故障状态文本 | 仅人工按需查询 |
| `Y` | SBUS 通道、九宫格、步态和控制源诊断文本 | 仅人工按需查询；当前代码可兼容小写 `y` |

- 二进制解析器优先。只有确认不属于二进制帧的独立字节才能进入 `p/Y` 诊断入口。
- `p/Y` 不改变模式、轴、session 或安全状态。
- 下位机不主动周期发送诊断文本；上电帮助和 CAN 原始日志默认关闭。
- 上位机自动任务不得轮询或解析 `p/Y` 文本来判断控制状态；它们只用于人工联调。
- 其他会改变电机或运动状态的历史单字符命令只允许在明确进入的维护模式或调试固件中使用。

## 7. 上位机 ROS2 映射

第一阶段最小数据流：

```text
任务状态机 / Nav2 / 视觉低速对准
  -> 唯一速度仲裁器
  -> motion_control_node
  -> 0x10，50 Hz
  -> STM32 USB/SBUS 仲裁
  -> 九宫格状态机、MIT 步态和轮驱
```

推荐字段映射：

| 上位机输入 | `0x10` 字段 |
| --- | --- |
| `linear.x` | `forward_permille` |
| `angular.z` | `yaw_permille` |
| 速度档/轮速限制 | `speed_permille` |
| 任务运动许可 | `DEADMAN_HELD + MOTION_ENABLE` |
| 正常停车 | 零轴 + `SMOOTH_STOP` |

8DOF 结构不支持真实横移，`linear.y` 必须为 0；非零时上位机应拒绝命令并停车。

第二阶段第二步数据流在此基础上增加：

```text
Mid-360 IMU
  -> 滤波
  -> 0x11，25 Hz
  -> STM32 DogImu_Update()
  -> 观察/姿态补偿（分阶段开启）
```

## 8. 机械臂 UART 文本协议

机械臂控制器使用 `115200 8N1`，命令以 `\n` 结尾：

| 命令 | 用途 |
| --- | --- |
| `HOME\n` | 回到待机位 |
| `VISION_SCAN\n` | 视觉扫描姿态 |
| `VISION_AID\n` | 辅助定位姿态 |
| `GRASP\n` | 抓取姿态 |
| `PLACE\n` | 放置姿态 |
| `SUCTION_ON\n` | 开启吸盘 |
| `SUCTION_OFF\n` | 关闭吸盘 |

动作完成确认、幂等 operation_id 和机构状态回传尚未在本协议定义。第一阶段任务赛联调必须通过机械臂控制器现有反馈或上位机超时策略保证动作不会无限等待。

## 9. 第一阶段验收

- 固定测试向量逐字节一致。
- `0x10` 发送频率为 `50 Hz +/- 2 Hz`。
- 分块、粘包、噪声和 CRC 错帧后能够恢复。
- payload 中出现历史 ASCII 调试字符不会触发串口命令。
- 松开任务运动许可后的下一帧为零轴。
- USB 拔线、上位机退出或 150 ms 无合法帧时，机器人不会保持最后一次非零运动。
- 实体 CH9、实体摇杆和模式开关可立即撤销 USB 控制。
- 不发送 `0x11` 时，任务赛底盘控制功能完整可用。

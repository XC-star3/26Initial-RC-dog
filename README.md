# RC-dog

RC-dog 是基于 STM32H723、LibXR 和 XRobot 的 8DOF 轮足四足机器人固件。当前工程只有一套运行架构：CubeMX 负责芯片外设初始化，LibXR 提供硬件抽象、线程和 XRUSB，项目生成器根据 IOC、LibXR 配置和 XRobot 模块清单生成平台及应用入口。

旧三任务入口、全局电机发送器、CAN/SBUS C 桥、在线调参、运动调试命令、固定 40 字节 USB 协议和 Cube CDC middleware 已删除。LibXR 子模块固定为提交 `c512b364ab1f0646bb86c4929ac5877e1bc7b62d`。

## 架构

| 模块 | 职责 | 执行上下文 |
| --- | --- | --- |
| `SbusReceiver` | 通过 LibXR `UART` 读取 UART5，解析 SBUS 并发布强类型快照 | HIGH，事件驱动 |
| `DogMotor` | 8 个 MW 腿电机、MIT PID、站立/低姿、运动学、足端轨迹和对角 trot | HIGH，500 Hz |
| `WheelMotor` | 4 个 C620/M3508、PI、斜坡、峰值电流预算、温度降额和反馈保护 | HIGH，500 Hz |
| `ObstacleController` | LOW/MID/HIGH 单级上台阶状态机 | 由控制线程推进 |
| `HostLink` | XRUSB CDC、严格 Topic packet 解析和状态发送 | MEDIUM，事件驱动/10 Hz 状态 |
| `RobotControl` | 唯一模式、SBUS/USB 仲裁和安全锁存所有者 | REALTIME，1 kHz |
| `StatusLED` | 通过 LibXR `SPI` 驱动 SPI6 板载 RGB，只显示系统状态 | LOW，10 Hz |

所有模块继承 `LibXR::Application`，使用 MANIFEST V2，并由构造函数接收 `HardwareContainer` 和 `ApplicationManager`。`OnMonitor()` 的周期为 10 Hz，不承载控制环；腿/轮模块只在此刷新反馈超时掩码，作为高频线程失效时的健康状态兜底。

硬件容器的稳定别名为：

| 别名 | 实体外设 | 用途 |
| --- | --- | --- |
| `fdcan_front` | FDCAN1 | 前腿 MW 电机 |
| `fdcan_rear` | FDCAN2 | 后腿 MW 电机 |
| `fdcan_wheel` | FDCAN3 | C620/M3508 轮电机 |
| `sbus_uart` | UART5 | 实体遥控器 |
| `usb_cdc` | USB OTG HS + XRUSB CDC | 上位机控制/状态 |
| `rgb_spi` | SPI6 | 板载 RGB |

生成的 `app_main()` 只构造 LibXR 平台对象、XRUSB 和硬件容器，然后调用生成的 `XRobotMain()`。FreeRTOS 只手工创建一个 8 KiB 静态启动任务；各模块线程栈由 `User/xrobot.yaml` 显式配置。FreeRTOS heap 为 48 KiB，CubeMX 的软件 timer task 未启用；`PlatformInit(2, 1024)` 的参数由 `User/libxr_config.yaml` 生成。

`dm02.ioc` 仍保留 FreeRTOS middleware 选择，但不再登记应用任务。`Core/Src/freertos.c` 的 native FreeRTOS 单启动任务和精简后的 `cmake/stm32cubemx/CMakeLists.txt` 由本工程维护，不在 CubeMX USER CODE 保护块内；使用 CubeMX 全量再生成后，必须复核并恢复这两个文件，避免重新引入 CMSIS-RTOS2 默认任务和已禁用的可选内核组件。

## LibXR 与 XRobot 生成

生成工具不锁定精确版本，最低要求为 `libxr>=5.2.4`、`xrobot>=0.3.1`。生成前会检查版本、命令行能力和当前 LibXR 头文件接口；兼容的新版本可以直接使用，缺少必要能力时会在写文件前失败。

```bash
python3 -m pip install -r tools/requirements-codegen.txt
python3 tools/generate.py
python3 tools/generate.py --check
```

唯一配置输入为 `dm02.ioc`、`User/libxr_config.yaml` 和 `User/xrobot.yaml`。生成器调用 LibXR IOC parser 和 STM32 emitter 能力检查，过滤出 FDCAN1/2/3、UART5、SPI6 与 USB OTG HS，再生成 `User/app_main.cpp`、`User/app_main.h` 和 `User/xrobot_main.hpp`。这三个文件带有 `AUTO-GENERATED` 标记，不得手工修改。

XRUSB 的端点缓冲、4096 字节硬件 FIFO 分配和 SPI6 非 DMA staging buffer 是 RC-dog 的结构化生成扩展，不依靠 C++ 文本替换。模块清单位于各模块头文件的 MANIFEST V2 块；本地模块索引为 `Modules/modules.yaml` 和 `Modules/sources.yaml`。

`User/codegen_manifest.cmake` 记录实际生成器版本、LibXR 提交和输入/输出 SHA256。CMake 配置阶段只使用自身哈希功能检查生成物，不会运行 Python 或修改源码；若显示 `Generated code is stale`，先重新运行统一生成命令。提交前应连续生成两次，并确认第二次无差异且 `--check` 通过。

首次获取工程后初始化固定的 LibXR 子模块：

```bash
git submodule update --init --recursive
git -C Middlewares/Third_Party/LibXR rev-parse HEAD
```

仓库当前记录的提交为 `c512b364ab1f0646bb86c4929ac5877e1bc7b62d`。生成器会记录实际提交并检查所需 C++ 接口，但不把该提交硬编码为生成条件。

## 构建与烧录

需要支持 C++20 的 GNU Arm Embedded 工具链、CMake 和 Ninja。工程只保留两个公开 preset：`Flash` 使用 Release 优化，`Debug` 保留调试符号。两者使用独立构建目录：

```bash
cmake --preset Flash
cmake --build --preset Flash -j4

cmake --preset Debug
cmake --build --preset Debug -j4
```

产物分别位于：

```text
build/Flash/RC-dog.elf
build/Flash/RC-dog.map
build/Debug/RC-dog.elf
build/Debug/RC-dog.map
```

查看固件段大小：

```bash
arm-none-eabi-size build/Flash/RC-dog.elf
arm-none-eabi-size build/Debug/RC-dog.elf
```

DAPLink/OpenOCD 配置为 `stm32h723_daplink.cfg`；CMSIS-DAP HID 回退配置为 `stm32h723_daplink_hid.cfg`。CMake、OpenOCD 和 `arm-none-eabi-gdb` 均从 `PATH` 查找。VS Code 的任务列表只显示 `Firmware: Flash` 和 `Firmware: Build Debug`；前者会依次完成 Flash 配置、构建和烧录，后者也是 `Firmware: Debug` 启动配置的预构建任务。命令行烧录示例：

```bash
openocd -f stm32h723_daplink.cfg \
  -c "init; reset halt; program build/Flash/RC-dog.elf verify; reset run; shutdown"
```

STM32Cube Build Analyzer 通过 CMake File API 发现构建产物。完成 Debug 构建后选择 `RC-dog.elf [Debug]`，分析器会读取同目录、同名的 `RC-dog.map` 并显示 FLASH/RAM；若仍显示历史构建记录，刷新 CMake 项目和 Build Analyzer 视图即可。

## SBUS 与模式

UART5 配置为仅接收的 `100000 8E2`，只使用 PD2 和 128 字节循环 RX DMA；未使用的 PC12 TX 已释放。通道使用从 0 开始的固件索引；遥控器面板通常以 CH1 开始编号。

| 遥控通道 | 固件索引 | 语义 |
| --- | --- | --- |
| CH1 | 0 | 转向/yaw |
| CH2 | 1 | 前进/后退 |
| CH3 | 2 | 轮速或步态速度；障碍模式档位 |
| CH5 | 4 | 九宫格主开关 |
| CH8 | 7 | 九宫格副开关 |
| CH9 | 8 | 实体安全输入，最高优先级 |
| CH10 | 9 | 单级上台阶触发 |

CH5/CH8 的 LOW/MID/HIGH 组合映射如下：

| 编号 | CH5 + CH8 | 模式 | 行为 |
| --- | --- | --- | --- |
| 0 | LOW + LOW | `MOTOR_CHECK` | 腿安全停止，轮关闭并锁定 |
| 1 | LOW + MID | `LOW_WHEEL` | 髋关节低姿 `-15 deg`，轮驱动 |
| 2 | LOW + HIGH | `LOW_WHEEL_REVERSE` | 髋关节反向低姿 `+5 deg`，轮驱动 |
| 3 | MID + LOW | `STAND_HOLD` | 站立，轮 HOLD |
| 4 | MID + MID | `STAND_WHEEL` | 站立完成后轮驱动 |
| 5 | MID + HIGH | `STAND_HOLD_ALT` | 保留站立，轮 HOLD |
| 6 | HIGH + LOW | `GAIT_ONLY` | 对角 trot，轮 HOLD |
| 7 | HIGH + MID | `GAIT_WHEEL` | 对角 trot 与轮驱动 |
| 8 | HIGH + HIGH | `OBSTACLE` | 仅实体 SBUS 可触发单级上台阶 |

低姿轮模式使用 `25%/12%` 进入/退出死区。运动方向反转前必须回中并保持 200 ms，轮目标会先减速到零再反向。CH3 将轮速上限连续映射到 `40..200 rpm`。

## XRUSB 接口

USB 使用 LibXR `STM32USBDeviceOtgHS + CDCUart`，不再包含 Cube CDC middleware。

| 项目 | 值 |
| --- | --- |
| VID / PID | `0x0483 / 0x5740` |
| 产品名 | `RC-dog XRUSB CDC` |
| 序列号 | STM32 96-bit UID 派生 |
| 速度/PHY | Full Speed / embedded PHY |
| CDC endpoints | EP1 OUT、EP1 IN、EP2 notification |

每个 Topic packet 为 `16 字节 header + payload + 1 字节尾 CRC8`：

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 1 | prefix，固定 `0x5A` |
| 1 | 3 | payload 长度，u24 little-endian |
| 4 | 4 | topic 名 CRC32，little-endian |
| 8 | 6 | 微秒时间戳，u48 little-endian |
| 14 | 1 | packet 版本，固定 `1` |
| 15 | 1 | 前 15 字节的 CRC8 |
| 16 | N | payload |
| 16+N | 1 | header 与 payload 的 CRC8 |

CRC8 初值为 `0xFF`、反射多项式 `0x8C`；CRC32 初值为 `0xFFFFFFFF`、反射多项式 `0xEDB88320`，不做最终异或。当前 topic 键为：

| Topic | 方向/频率 | CRC32 |
| --- | --- | --- |
| `rcdog.control.command.v1` | PC -> 固件，50 Hz | `0xBBA1BC07` |
| `rcdog.status.v1` | 固件 -> PC，10 Hz | `0xC42B732F` |

### ControlCommandV1

payload 固定 24 字节、小端：

| 偏移 | 类型 | 字段 | 范围 |
| --- | --- | --- | --- |
| 0 | `u8` | `schema_version` | 固定 1 |
| 1 | `u8` | `mode` | `0..8` |
| 2 | `u16` | `flags` | bit0 deadman、bit1 motion-enable、bit2 smooth-stop |
| 4 | `i16` | `yaw` | `-1000..1000` |
| 6 | `i16` | `forward` | `-1000..1000` |
| 8 | `i16` | `speed` | `0..1000` |
| 10 | `u16` | `reserved` | 固定 0 |
| 12 | `u32` | `session_id` | 非 0 |
| 16 | `u32` | `command_counter` | 同一 session 单调前进 |
| 20 | `u32` | `host_time_ms` | 主机单调时钟低 32 位 |

USB mode 8 是保留站立，不会进入越障。固件严格拒绝错误版本、非精确长度、未知 flag、非零 reserved、越界轴、零 session、topic/CRC 错误；不完整包超过 50 ms 后重同步。

### RobotStatusV1

payload 固定 24 字节、小端：

| 偏移 | 类型 | 字段 |
| --- | --- | --- |
| 0..11 | 12 x `u8` | 版本、控制源、请求/激活模式、进入状态、阻塞原因、安全锁、障碍状态/故障、腿/轮在线掩码、reserved |
| 12 | `u32` | `fault_bits` |
| 16 | `u32` | `last_command_counter` |
| 20 | `u32` | `uptime_ms` |

`fault_bits` 从 bit0 起依次为：SBUS 丢失、USB 丢失、腿离线、腿驱动故障、轮离线、轮过温、CAN bus-off、障碍故障、安全锁存、USB 协议错误。

## 输入仲裁与安全

- 上电默认安全锁存；没有新鲜 SBUS 时不允许运动。
- SBUS 新鲜度为 250 ms，signal-lost/failsafe 位同样会使输入无效。
- CH9 高位立即锁存安全、停止腿部、轮电流清零并锁轮；只有 CH9 释放、九宫格回到 LOW+LOW 并稳定 150 ms 才解除锁存。
- 实体 SBUS/CH9 始终高于 USB。USB 只有在实体开关 LOW+LOW 且 CH1/CH2 位于中位时才可能取得控制权。
- 新 USB session 必须先连续发送 3 帧 mode 0、flags 0、三个运动轴全 0 的安全命令；被撤销的 session 不可直接重新获取。
- USB 命令看门狗为 150 ms；轮控制命令看门狗为 100 ms；腿/轮反馈和 CAN bus-off 会进入故障状态。
- USB 看门狗触发后 `FAULT_USB_LOST` 保持到新的三帧安全零命令会话成功建立；旧 session 不能直接恢复运动。
- smooth-stop 会把 gait 受控切回标准站姿，同时将混合模式轮目标降为零；缺少 deadman 或 motion-enable 时腿和轮都不能接收运动目标。
- 腿控制进入 SAFE/FAULT 时向全部 8 个节点发送 Axis State Idle，并以 100 ms 周期重试；CAN 接收与控制线程通过临界区快照隔离。
- 轮电机 70 C 开始降额，85 C 切断并锁存，降至 65 C 才允许恢复；连续电流 6 A、峰值 10 A、峰值预算 500 ms。
- `MOTOR_CHECK`、输入超时和安全锁存不能通过遗留文本命令、旧 USB 帧或调试命令旁路。

## 电机映射与控制参数

腿部 FDCAN 使用经典 CAN；数组顺序是稳定控制编号：

| 索引 | 腿/关节 | 总线/节点 | 编码方向 | 转矩方向 | 传动比 | 用户角范围 |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | LF hip | FDCAN1 / 2 | +1 | -1 | 8:1 | -120..120 deg |
| 1 | LF knee | FDCAN1 / 1 | -1 | +1 | 8:1 | -120..143 deg |
| 2 | RF hip | FDCAN1 / 4 | -1 | +1 | 8:1 | -120..120 deg |
| 3 | RF knee | FDCAN1 / 3 | +1 | +1 | 8:1 | -120..143 deg |
| 4 | LB hip | FDCAN2 / 2 | +1 | -1 | 8:1 | -120..120 deg |
| 5 | LB knee | FDCAN2 / 1 | -1 | -1 | 8:1 | -120..143 deg |
| 6 | RB hip | FDCAN2 / 4 | -1 | -1 | 8:1 | -120..120 deg |
| 7 | RB knee | FDCAN2 / 3 | +1 | +1 | 8:1 | -120..143 deg |

MIT 角度 PID 为 `Kp=1.7 A/deg`、`Ki=0`、`Kd=0.017 A/(deg/s)`；电流上限 18 A、速度上限 30 turn/s、转矩常数 1.2 Nm/A。前腿站立足端为 `(0,300) mm`，后腿为 `(0,315) mm`；trot 使用 LF+RB 与 RF+LB 对角相位，频率上限 2 Hz，标准摆动净空 70 mm。

轮电机使用 FDCAN3，反馈 ID `0x201..0x204`，控制 ID `0x200`。槽位前进符号依次为 `[-1,+1,+1,-1]`，M3508 传动比 `268/17`。速度 PI 为 `Kp=0.015`、`Ki=0.25/s`，加速/减速/制动斜坡为 `30/50/60 rad/s^2`。

## 单级上台阶

只有实体 SBUS HIGH+HIGH 能运行 `ObstacleController`。USB mode 8 始终是保留站立。

| CH3 档位 | 落脚高度 | 摆动净空 | 总抬升 | 机身预升 |
| --- | --- | --- | --- | --- |
| LOW | 50 mm | 70 mm | 120 mm | 0 mm |
| MID | 100 mm | 70 mm | 170 mm | 0 mm |
| HIGH | 150 mm | 70 mm | 220 mm | 30 mm |

进入模式后档位锁存。轮必须全部在线并停止，HOLD 连续 200 ms 后才允许请求；CH10 初始位置只同步，任一方向稳定切换 8 个新鲜 SBUS 帧触发一次完整动作。动作期间 CH2 无效。

LOW/MID 共 25 个足端分段，HIGH 因 30 mm 机身预升为 26 段。生产序列为：机身准备、四腿移到 `X=-108 mm`、左右后腿依次紧凑到 `X=+108 mm`；左前/右前分别按“抬升 1000 ms、前送 1500 ms、落脚 1800 ms”上台；机身移重 2000 ms；左后/右后执行相同三段；机身恢复 1800 ms；前腿、机身、后腿依次前送并归一到标准站姿。水平踏面前送按 300 mm 设计，顶部动作使用 30 mm 平地净空。

普通模式切出时不抢占正在执行的足端分段；当前分段完成后暂停，重新进入 HIGH+HIGH 从下一段继续。CH9 或 SBUS 失效会立即安全中止。当前没有足底/落轮传感器，轨迹完成只说明命令位置完成，不能证明轮胎已正确落到台面；不实现下台阶、连续多级、木桥 B、砂砾坑、虚拟 IMU 或逆序回退。

每个足端分段在提交前按 32 个区间离散预检完整插值轨迹、关节角范围和逆运动学可达性；500 Hz 运行期再次检查 IK。预检失败记为 `MOTION_START`，运行期失败锁存腿驱动故障、停止序列并将所有腿节点切入 Idle。

## 主机 CLI

主机只依赖 `pyserial`：

```bash
python3 -m pip install -r host/requirements.txt
```

打开串口后，发送线程先自然产生至少 3 帧安全零命令。示例：

```bash
# 只建链、保持安全零命令并读取结构化状态
python3 -m host.quadruped_control_cli /dev/ttyACM0 --duration 0.2 --status

# 站立轮行 2 秒，forward/yaw/speed 范围均为归一化值
python3 -m host.quadruped_control_cli /dev/ttyACM0 \
  --mode STAND_WHEEL --forward 0.25 --yaw 0.0 --speed 0.4 \
  --enable --deadman --duration 2 --status

# 请求平滑停止
python3 -m host.quadruped_control_cli /dev/ttyACM0 \
  --mode GAIT_ONLY --enable --deadman --smooth-stop --status
```

Windows 端口示例为 `COM7`。遗留固定帧、文本诊断命令和 ROS2 包不再支持；主机集成应复用 `host/xrusb_codec.py` 的 Topic codec。

## 故障排查

| 现象 | 检查项 |
| --- | --- |
| USB 不枚举 | 确认设备名、VID/PID、PA11/PA12、HS PCD Full Speed 配置和 OTG_HS IRQ；工程中不应再链接 Cube CDC middleware |
| USB 命令无效 | SBUS 必须新鲜，CH9 已释放，CH5/CH8 为 LOW+LOW，CH1/CH2 居中；使用新 session 连续发送三帧安全零命令 |
| `FAULT_USB_PROTOCOL` | 核对 topic CRC、24 字节 payload、little-endian、版本/flags/reserved、双 CRC8 和 50 ms 分包间隔 |
| 腿电机离线 | 检查 FDCAN1/2、节点映射、心跳/编码器帧和 8:1 编码换算；确认 map 中未发生内存溢出 |
| 轮电机无法解锁 | 四轮反馈掩码必须为 `0x0F`、速度小于 0.5 rad/s、无过温和 bus-off |
| 楼梯停在 PRECHECK | 确认腿/轮全部健康、标准站立完成、轮停止并持续 200 ms |
| RGB 不亮 | 检查 SPI6 PA5/PA7、mode 0/edge 2、prescaler 4 和板载 RGB 数据方向 |
| CMake 提示生成物过期 | 安装最低版本生成依赖，运行 `python3 tools/generate.py`，再用 `--check` 确认输入、输出和清单一致 |

## 重构记录与优势

本次重构将混合工程变为单一 LibXR/XRobot 路径：

- 删除旧三任务入口、全局电机状态、HAL 回调转发桥和重复 USB CDC 栈，CAN/UART/SPI/USB 统一通过 LibXR 驱动抽象。
- `RobotControl` 成为唯一输入仲裁和安全所有者，SBUS、USB、模式进入条件和故障状态不再散落在多个任务。
- 电机控制、越障、主机链路和状态灯具有明确依赖边界，线程频率和栈大小可从 XRobot 配置审查。
- LibXR 平台适配、XRUSB、硬件别名和应用入口均可重复生成，消除 IOC、缓冲参数与手工实例化之间的漂移。
- 二进制状态 Topic 取代不可机读的文本诊断，上位机能观察控制源、模式进入阻塞、在线掩码和故障位。
- Cube USB Device middleware 已移除，避免 ST CDC 回调与 XRUSB PCD 回调同时拥有同一外设。
- 模块源码和应用入口在 CMake 中显式列出，移除了重复目标传播、无效编译定义以及当前配置禁用的 FreeRTOS coroutine、MPU wrapper、event group 和 stream buffer 编译单元。
- 删除了只写不读的电机反馈/计数、重复命令状态和陈旧 CubeMX 三任务元数据；保留腿轮在线超时兜底，模式、安全锁、PID、限幅、热保护与 CAN 行为不变。
- HostLink 将命令、接收时间和代际作为同一临界区快照发布，并复用线程生命周期同步对象；50 ms 半包超时在消费新字节前执行，上位机重连时也会清空旧解析半包。

代码量按物理行统计，不包含第三方库、Cube/HAL 平台代码和生成的三个入口文件：重构前取提交 `67dc029` 中 `User_File + host` 的 C/C++/Python 文件，重构后取当前 `Modules + User + host + tools` 中的手写业务及生成工具源码。

| 指标 | 重构前 | 重构后 |
| --- | --- | --- |
| 手写业务与生成工具源码物理行 | 16,650 | 4,356 |
| LibXR/XRobot 生成入口 | 无 | 自动生成（不计入业务源码） |
| Debug 链接器 FLASH | 152,080 B（迁移过程中的旧/新混合基线） | 124,536 B |
| Debug 链接器 DTCMRAM | 78,456 B（混合基线） | 95,752 B |
| Debug `size` text/data/bss | 未保留可复现旧产物 | 124,384 / 148 / 95,720 B |
| Flash（Release）链接器 FLASH / DTCMRAM | 未保留可复现旧产物 | 99,836 / 95,744 B |
| Flash（Release）`size` text/data/bss | 未保留可复现旧产物 | 99,688 / 144 / 95,712 B |

这些数据只描述当前构建和代码规模，不代表控制性能、实时性或实机可靠性提升。

## 验收状态

自动验收包括：统一生成连续幂等、`--check`、CMake 过期拒绝、干净 Flash/Debug 配置与链接、`arm-none-eabi-size`、链接 map 内存区域检查、Python `compileall`、XRUSB codec 回归、静态检查，以及旧目录/协议/文档引用扫描。本次交付不恢复旧业务单元测试，编译仍是阻塞验收项。

以下项目必须在架空、防跌落和可直接急停的条件下人工回归；当前重构未进行实机验证，不得视为已完成：

- [ ] 上电安全和默认安全锁存
- [ ] CH9 急停、释放条件和重复触发
- [ ] SBUS 250 ms / USB 150 ms 超时
- [ ] CH5/CH8 九宫格全部模式及低姿换向中立等待
- [ ] 三条 CAN 的 bus-off 停止与恢复
- [ ] 轮电机 70/85/65 C 降额、切断和恢复
- [ ] 对角 trot、停止和轮腿混合
- [ ] LOW/MID/HIGH 单级上台阶完整序列、中断续跑和 CH10 八帧触发

## 贡献者

| 贡献者 | 主要贡献 | 个人主页链接 |
| --- | --- | --- |

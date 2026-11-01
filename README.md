# RoboCon 2026 排球机器人前车板固件

基于 **STM32F427IIHx（Cortex-M4F）**、STM32 HAL 和 FreeRTOS 的前车板控制工程，负责击球平台、云台、底盘指令发送，以及遥控和上位机通信。工程名与编译目标均为 `testa1`。

## 主要功能

- **击球平台**：三台 A1 电机控制，包含初始位姿、接发击球、颠球、停止等模式，以及轨迹生成、运动学解算和力矩输出。
- **云台**：Yaw / Pitch 双轴位置与速度串级 PID，支持遥控目标和上位机目标；初始化状态为 `Gimbal_Off`。
- **底盘协同**：通过 CAN2 发送底盘状态与速度指令，包含基于 DT35 测距反馈的定位逻辑。
- **通信**：通过 UART DMA 接收遥控、云台目标、击球触发和底盘反馈，并向上位机发送云台反馈。

## 工程结构

```text
.
├── Core/Inc/、Core/Src/   头文件与源代码，按以下层次组织
│   ├── Application/      初始化及 FreeRTOS 业务任务
│   ├── Modules/          击球平台、底盘等模块逻辑
│   ├── Communicate/      上位机协议收发与数据解析
│   ├── Periphal/         电机、遥控器、DT35 驱动（沿用原目录拼写）
│   ├── Algorithm/        PID、滤波、CRC 等算法
│   ├── System/           控制参数、串口绑定、DWT 计时
│   └── Utility/、Library/ 外设封装与通用工具
├── Drivers/              STM32 HAL 与 CMSIS
├── Middlewares/          FreeRTOS 与 CMSIS-RTOS 接口
├── MDK-ARM/              Keil 工程、启动文件与 EIDE 配置
├── patch/                AC5 / AC6 的 FreeRTOS 移植层切换脚本
└── testa1.ioc            STM32CubeMX 配置
```

启动入口为 [main.c](Core/Src/main.c)：外设初始化 → [Init_InitAll()](Core/Src/Application/app_init.c) → 创建任务 → 启动调度器。[freertos.c](Core/Src/freertos.c) 创建初始化、平台、通信、遥控、底盘和云台六个任务，业务实现在 `Application/` 中。

## 编译与下载

### Keil MDK

1. 安装 Keil MDK 5、**Arm Compiler 6** 和 STM32F4 器件支持包。工程记录的版本为 **Arm Compiler 6.19**、`Keil.STM32F4xx_DFP.2.17.1`。
2. 确认本地存在 `MDK-ARM/arm_cortexM4lf_math.lib`。工程链接此 CMSIS-DSP 库，但该文件未纳入 Git，首次克隆需从原开发环境或匹配的 CMSIS-DSP 库包补齐。
3. 打开 [MDK-ARM/testa1.uvprojx](MDK-ARM/testa1.uvprojx)，选择 `testa1` 目标，执行 **Rebuild**。输出为 `MDK-ARM/testa1/testa1.axf` 和 `testa1.hex`。
4. 在工程的 Debug / Flash Download 设置中选择实际使用的调试器，核对芯片和下载算法后下载。

HAL、CMSIS 和 FreeRTOS 源码已随仓库提供。只有修改外设配置或重新生成代码时才需要 CubeMX；`testa1.ioc` 记录的版本为 CubeMX **6.15.0**、STM32CubeF4 **1.28.3**。

### VS Code + EIDE

打开 `MDK-ARM/` 文件夹，使用 [EIDE 配置](MDK-ARM/.eide/eide.yml)，配置本机 AC6 工具链及 J-Link。可通过已有的 `build`、`rebuild`、`flash`、`build and flash` 任务编译或下载，输出目录为 `MDK-ARM/build/`。

**EIDE 编译前需处理移植层差异**：EIDE 配置引用 `portable/RVDS/ARM_CM4F`，该目录当前为 AC5 汇编写法。使用 AC6 时，在仓库根目录执行：

```powershell
.\patch\patchAC6.bat
```

脚本会覆盖对应的 FreeRTOS `port.c` 和 `portmacro.h`。Keil 工程已引用 `portable/GCC/ARM_CM4F`，无需执行此补丁。

## 硬件与通信配置

当前代码使用 **12 MHz 外部晶振、168 MHz 系统时钟**。串口绑定见 [sys_const.c](Core/Src/System/sys_const.c)，引脚和串口格式见 [usart.c](Core/Src/usart.c)，CAN 配置见 [can.c](Core/Src/can.c)。

- **USART6，4,800,000 baud**：平台电机通信。当前平台输出使用三台 A1，前 / 左 / 右 ID 分别为 `1 / 0 / 2`；驱动中同时保留 Go1 等电机支持。
- **CAN1，1 Mbit/s**：保留 DM4340 电机组配置，发送 ID 为 `0x01–0x03`，反馈 ID 为 `0x04–0x06`；当前 `Platform_Output()` 发送 A1 指令。
- **CAN2，1 Mbit/s**：云台电机及底盘协同。云台发送 ID 为 `0x2FE`，Yaw / Pitch 反馈 ID 为 `0x209 / 0x20A`；底盘控制帧 ID 为 `0x07`。
- **UART7，115,200 baud，8N1**：上位机通信，接收帧长为 **55 字节**，帧头 `5A A5`、帧尾 `7A A7`。字段定义以 [comm_receive.c](Core/Src/Communicate/comm_receive.c) 和 [comm_transmit.c](Core/Src/Communicate/comm_transmit.c) 为准。
- **UART8，115,200 baud，8N1**：DT35 测距数据。
- **USART1，100,000 baud**：独立遥控接收接口；当前 `Remote_ControlCom()` 使用的是 UART7 协议解析出的 `Remote_Data`，模式由 `s[0]`、`s[3]` 决定。

## 常用修改入口

- 平台击球状态机、轨迹与输出：[module_platform.c](Core/Src/Modules/module_platform.c)。
- PID、机构尺寸、云台零偏、串口绑定：[sys_const.c](Core/Src/System/sys_const.c)。
- 电机型号、ID、编解码：[periph_motor.c](Core/Src/Periphal/periph_motor.c)。
- 遥控模式映射：[app_remote.c](Core/Src/Application/app_remote.c)；云台控制：[app_gimbal.c](Core/Src/Application/app_gimbal.c)。
- 底盘控制与 CAN 打包：[module_chassis.c](Core/Src/Modules/module_chassis.c)。
- 协议帧校验与分发：[comm_common.c](Core/Src/Communicate/comm_common.c)。

重新生成 CubeMX 代码后，检查自定义初始化、任务入口及 FreeRTOS 移植层配置。已有注释、PID 说明和流程图可能保留旧版本行为，修改时以当前编译目标引用的代码为准。

调试前核对电机 ID、零偏和机构限位：`Init_InitAll()` 会发送 DM 电机使能指令，平台任务中的 `Platform_Check()` 调用目前被注释，独立的离线检测函数也不等同于已启用的停机保护。

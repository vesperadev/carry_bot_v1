# carry_bot_v1

STM32F407 控制固件工程。STM32CubeMX 生成外设配置，CMake/Ninja 构建，FreeRTOS（CMSIS-RTOS V2）调度，用户代码以 C++ 编写。

> 本文件记录**当前实现状态与开发进度**。开发按「完成一点、验证一点、提交一点」推进，进度随提交更新。
> 最近更新：2026-11-09

## 1. 平台与工具链

| 项 | 值 |
|---|---|
| MCU | STM32F407IGH6，Cortex-M4F @ 168 MHz |
| HAL | STM32Cube FW_F4 V1.28.3 |
| RTOS | FreeRTOS（heap_4，CMSIS-RTOS V2），`configTOTAL_HEAP_SIZE = 15360` |
| 编译 | arm-none-eabi-gcc 14.3.1 + CMake 4.3.1 + Ninja |
| 构建目录 | `build/Debug`（`build/` 已被 `.gitignore` 忽略） |
| 串口 | USART6：PG14/TX、PG9/RX，9600；USART1：PB6/PB7，115200 |
| DMA | USART6 RX/TX = DMA2_Stream1 / DMA2_Stream6，Channel 5，Normal 模式 |
| 时钟基准 | TIM7 提供 HAL timebase，SysTick 交 RTOS |

## 2. 目录结构

```
Core/                    CubeMX 生成层：外设初始化、中断向量、FreeRTOS 初始化
  Src/main.c             仅保留 CubeMX 流程，业务初始化已下放到任务
  Src/freertos.c         USER CODE 段中创建 linkTask
Drivers/ Middlewares/    F4 HAL、CMSIS、FreeRTOS 源码（CubeMX 生成，不手改）
cmake/                   CubeMX 生成的构建清单 + 工具链文件
app/Link/                USART6 上位机串口链路模块（唯一自研模块）
  link_public.h          对外契约（extern "C"）
  link_config.h          配置常量与任务参数
  link_protocol.h        LinkFrame + LinkProtocol 纯虚接口
  link.cpp               驱动层：任务、队列、IDLE/错误回调、发送
  link_proto_debug.cpp   BT 对端（蓝牙调试）：收到即回显
  link_proto_rpi.cpp     RPI 对端（树莓派）：CRC8 帧 + PING/ACK 骨架
  CMakeLists.txt         INTERFACE 库，按 LINK_PEER 选编协议实现
```

分层约定：生成层零侵入（业务只写在 `USER CODE` 段）；一个模块 = 一个目录 = 一个 CMake target；驱动层 / 协议层 / 业务层分离，换对端只换协议实现文件。

## 3. 当前能做什么

- 上电后 `linkTask` 自动建队列、启动 `HAL_UARTEx_ReceiveToIdle_DMA`，USART6 按总线空闲分帧；单帧上限 256 B，DMA 缓冲 1024 B，队列深度 4。
- **BT 对端（默认，9600）**：收到任意字节原样回显，用于验证蓝牙透传、接线、波特率与 MCU 收发链路。
- **RPI 对端（`-DLINK_PEER=RPI`，115200）**：`0xA5 | cmd | len | payload | crc8` 帧校验 + PING/ACK 回复骨架，业务命令待定。
- 底层能力：关闭 DMA HT 中断防半帧误判、串口错误后自动重启接收、发送前检查 `gState`/DMA 忙并拷贝到静态发送缓冲；对外提供 `Link_Send()`、`Link_SetBaudRate()`（当前无其它模块调用）。

边界：不配置蓝牙模块本身（AT/配对/连接状态），不解析任何业务命令，不主动上报，不把帧转发给其它任务；无校验、重传、流控，超长帧或队列满会丢弃。**仅编译验证通过，尚未烧录和硬件实测。**

最近一次构建（Debug，`LINK_PEER=BT`）：RAM 22784 B / 128 KB（17.38%），FLASH 约 36 KB / 1 MB（3.52%），零警告。

## 4. 构建

```powershell
$cmake = "C:\Users\37963\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe"

& $cmake -S . -B build/Debug -DLINK_PEER=BT     # 默认：蓝牙调试对端
& $cmake --build build/Debug --

& $cmake -S . -B build/Debug -DLINK_PEER=RPI    # 切换到树莓派对端
& $cmake --build build/Debug --
```

`LINK_PEER` 由 `app/Link/CMakeLists.txt` 定义，选编 `link_proto_debug.cpp` 或 `link_proto_rpi.cpp`，并向 `link_config.h` 注入 `LINK_PEER_BT` / `LINK_PEER_RPI`。两种对端均已全量编译链接通过。

## 5. 开发进度

### 已完成

| 阶段 | 内容 | 状态 | 提交 |
|---|---|---|---|
| 工程基线 | CubeMX F407 + FreeRTOS + CMake 工程纳入 git，建立 `main` 基线 | 完成 | `e0b2525` |
| 链路驱动 | `app/BT` 初版：DMA+IDLE 收、队列、任务、IDLE/错误回调、DMA 发送 | 完成 | `e0b2525` |
| 协议分层 | `parse()` / `make_reply()` 纯虚接口 + BT/RPI 两套实现 + CMake 选编 | 完成 | `e0b2525` |
| 命名修正 | `app/BT` → `app/Link`，`LINK_PEER=BT/RPI`，定位改为「USART6 上位机串口链路」 | 完成 | `a734437` |
| 编译验证 | 两种对端零警告通过，旧命名零残留，lint 无错误 | 完成 | — |
| 参考工程分析 | `CubeMx/USART`（F103 裸机）模块逻辑、冲突清单、移植策略（只读，未改代码） | 完成 | — |

### 进行中

| 阶段 | 内容 | 状态 |
|---|---|---|
| USART 模块移植 | 差异分析、命名冲突清单、阶段划分已完成，**代码尚未落地** | 进行中 |

### 待办（按小步提交顺序）

| # | 阶段 | 内容 | 验证方式 |
|---|---|---|---|
| 1 | 协议层落地 | 核对 `.ioc` 中 USART1 引脚/DMA Stream/NVIC；新建 `app/ZDT` 抽出 Emm V5 帧格式与命令解析；BT 对端加 `M` 命令，回显组帧 HEX | 编译 + 回显帧与 `COMMANDS.md` 帧例逐字节一致 |
| 2 | 统一回调分发 | `HAL_UARTEx_RxEventCallback` / `HAL_UART_ErrorCallback` 收敛为一份，按 `huart->Instance` 分发，为 USART1 预留 | 回显回归测试 |
| 3 | 电机总线驱动 | CubeMX 配 USART1 DMA/中断，新建 `app/ZDT` 链路驱动（复用 `link.cpp` 骨架改 `huart1`） | 单独验证电机总线收发 |
| 4 | 桥接任务 | 新建 `app/Bridge`：蓝牙命令 → Emm V5 帧 → 电机应答回蓝牙 | 端到端联调 |
| 5 | 状态机与外设 | 麦轮运动学、地址管理、回零、PCA9685、OLED | 分模块验证 |
| 6 | 硬件联调 | 蓝牙回显、电机动作、整车上电自检 | 实车验证 |

### 已知风险

- 未来接第二路串口时，`HAL_UARTEx_RxEventCallback` 全工程只能有一份强定义（参考工程 `Core/Src/main.c:2066` 也定义了同名函数），必须先合并分发。
- 参考工程 NVIC 优先级为 0，本工程在 RTOS 下须为 5~15；其轮询发送 + `HAL_Delay` 必须改成 DMA + 发送队列。
- 参考工程默认开启 `BOOTZERO_ENABLE` / `WHEEL_SELFTEST_ENABLE`，通电会自动发帧转轮子，移植前须关闭。
- `Core/Src/Backup/*.bak` 等备份文件目前也在版本库内，可按需补 `.gitignore`。

## 6. 版本管理约定

- 当前分支 `feature/usart-module-port`；`main` 只保留可编译基线。
- 参考工程 `CubeMx/USART` 是独立 git 仓库，使用同名分支；该仓库当前只做只读参考。
- 提交节奏：完成一点、验证一点、提交一点；使用 `feat:` / `refactor:` / `fix:` / `docs:` 前缀，不攒大提交，不提交 `build/`。

## 7. 变更记录

| 日期 | 内容 |
|---|---|
| 2026-11-09 | 初始化工程基线；建立串口链路模块与可插拔协议层；模块更名为 `app/Link`；补充本 README 与进度表 |

# Servo / PCA9685 移植计划与实现说明

> 分支：`feature/servo-pca9685`  
> 更新：2026-11-09  
> 参考：`CubeMx/I2C`（F103 调参工程）、`docs/PCA9685`、`USART/COMMANDS.md` §5、`docs/DEVELOPMENT.md`

## 1. 目标

在 `carry_bot_v1`（F407 + FreeRTOS）落地 `app/Servo`：

- 总线：`I2C1` → PCA9685（地址 `0x40`）
- 通道：`CH0` 机械爪夹紧 / `CH1` 物料转盘
- 对 Bridge 暴露同步友好的 C API；**不**在本分支解析蓝牙 `S*` 命令（属 `feature/bridge-bt-commands`）

## 2. 硬件（本工程 .ioc）

| 信号 | F407 引脚 | 备注 |
|---|---|---|
| I2C1_SCL | **PB8** | Fast 400 kHz，开漏 AF4 |
| I2C1_SDA | **PB9** | 同上 |
| DMA RX | DMA1 Stream0 / CH1 | `I2C1_RX` |
| DMA TX | DMA1 Stream6 / CH1 | `I2C1_TX` |
| NVIC | I2C1_EV 优先级 **5** | 符合 RTOS 5–15 |

> 对照：`CubeMx/I2C`（F103）用的是 **PB6/PB7**。上板接线按 **本仓 PB8/PB9**，不要照抄 F103 丝印。

PCA9685：

- 7-bit 地址 `0x40`（HAL 写地址 `0x80`）
- `VCC` 逻辑 3.3 V；`V+` 舵机独立 5–6 V 大电流；必须共地
- OE 悬空/拉低按模块说明

## 3. 寄存器要点（手册 + I2C 工程）

| 项 | 值 / 步骤 |
|---|---|
| MODE1 | `0x00`；改 PRESCALE 前进入 SLEEP（bit4） |
| PRESCALE | `0xFE`；50 Hz → `round(25e6/(4096*50)-1)=121` |
| 唤醒 | 写回 MODE1 → delay ≥500 µs → `old\|0xA1`（RESTART\|AI\|ALLCALL） |
| LEDn | `LED0_ON_L=0x06`，每通道 4 字节 ON_L/ON_H/OFF_L/OFF_H |
| 脉宽 | `counts = us * 4096 * freq / 1e6`，`SetPWM(ch,0,counts)` |

脉宽钳位探查范围 **200–2800 µs**（兼容 270°）；夹爪有效工作区仍是 **1000（合）–1500（开） µs**（2026-11-08 校准）。

## 4. 软件架构

```
Bridge (later)  --Servo_Set*-->  queue  -->  servoTask  --mutex-->  I2C1 Mem_*_DMA  --> PCA9685
Link / ZDT      (USART DMA)                不碰 I2C 回调
```

- **唯一** I2C 完成回调：`HAL_I2C_MemTxCpltCallback` / `MemRxCpltCallback` / `ErrorCallback`（仅 `Instance==I2C1`），放在 `pca9685.cpp`
- 总线 mutex + 完成二进制信号量；禁止在 ISR 里直接 `Mem_Write`
- UART 回调仍由 Link（及日后合入的 ZDT 分发）独占；Servo **零** UART 钩子
- `SERVO_REQUIRE_DEVICE=1`：初始化 `IsDeviceReady` 失败则 `Servo_IsReady()==0`，任务软挂起，不 `Error_Handler`

### 公共 API（`servo_public.h`）

| API | 作用 |
|---|---|
| `StartServoTask` | FreeRTOS 入口：建队列、Init、可选上电摆位、消费命令 |
| `Servo_IsReady` | PCA9685 是否在线 |
| `Servo_SetPulseUs(ch,us)` | 投递脉宽命令（Bridge 主路径） |
| `Servo_SetClampUs` / `SetTurntableUs` | CH0 / CH1 语法糖 |
| `Servo_ClampClose/Open` / `TurntableMid` | 校准预设 1000 / 1500 / 1500 |
| `Servo_GetPulseUs` | 最近成功写入的脉宽缓存 |
| `Servo_SetPWM` | 底层 ON/OFF counts（调试） |

## 5. 与 Bridge `S` 命令的衔接（本分支不做解析）

权威语法见 `USART/COMMANDS.md` §5，例如：

- `S0U1000` / `S0U1500` → `Servo_ClampClose` / `Open` 或 `Servo_SetClampUs`
- `S1Uxxxx` → `Servo_SetTurntableUs`
- `S1C/S/D/T/A` 摆动调参：后续可选 `app/Servo` 扩展或留在 Bridge；**当前驱动只保证绝对脉宽**

验收（联调阶段）：

1. 烧录后 `Servo_IsReady()==1`（可临时用 Link 回显或调试器）
2. 调用 `Servo_ClampClose` / `Open` 夹爪动作与 I2C 工程一致
3. `Servo_SetTurntableUs` 转盘有响应（CH1 端点仍待 270° 实测）
4. 与 ZDT/Link 同跑无 HardFault；I2C 与 USART DMA 流不冲突（本 .ioc 已分开）

## 6. CubeMX / RTOS 风险清单

| 风险 | 对策 |
|---|---|
| 重生工程丢 I2C DMA | 核对 Stream0/6 与 `__HAL_LINKDMA` |
| I2C1_ER 未开 | 现仅 EV；超时路径 `Master_Abort_IT`；若 NACK 挂死再开 ER |
| 回调重复定义 | Servo 独占 I2C 回调；其他模块勿再写 `HAL_I2C_*Callback` |
| 阻塞 `HAL_Delay` | 驱动内改 `vTaskDelay` |
| 与 ZDT 合入冲突 | 本分支不改 `app/ZDT` / UART 分发；合 main 时只冲突 `freertos.c` USER 区 |

## 7. 文件一览

| 路径 | 说明 |
|---|---|
| `app/Servo/pca9685.*` | 寄存器 + DMA 传输 |
| `app/Servo/servo.*` | 任务、队列、通道语义 |
| `app/Servo/servo_config.h` | 地址/通道/校准宏 |
| `app/Servo/servo_public.h` | Bridge 可用 API |
| `Core/Src/freertos.c` | 创建 `servoTask` |
| `CMakeLists.txt` | `add_subdirectory(app/Servo)` + `servo` |

## 8. 未决问题

1. CH1 转盘 270° 端点脉宽尚未在实车上标定（I2C 工程记「待 11-09 试」）
2. 是否在 Servo 内做自动摆动（`A/C/S/D/T`），还是只留绝对 `U`、摆动由 Bridge 定时调用？
3. 是否启用 `I2C1_ER_IRQn` 以完善 NACK/仲裁丢失回调？
4. 上电默认：当前 `SERVO_BOOT_APPLY=1` 夹爪开、转盘中；量产是否改为「无输出直到首条命令」？
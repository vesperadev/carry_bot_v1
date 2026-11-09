# 智能物流搬运 · 开发说明

> 更新：2026-11-09 · 协调人：Mini_Agent  
> 目标仓库：`CubeMx/carry_bot_v1`（F407 + FreeRTOS 干净重构）  
> 成功但杂乱的参考：`CubeMx/USART`（F103，蓝牙可控实车，只读）  
> 资料根：`Intelligent_Logistics_Handling/`（顶层暂未建 monorepo git）

## 1. 项目地图

| 路径 | 角色 | 状态 |
|---|---|---|
| `CubeMx/carry_bot_v1` | **正式开发主线**（STM32F407IGH6） | Link+ZDT+Servo+Bridge A-tier（待合 main） |
| `CubeMx/USART` | 蓝牙→Emm V5 成功案例（F103） | 可用但乱；命令见 `COMMANDS.md`；**只读参考** |
| `CubeMx/I2C` | PCA9685 + 部分步进调参 | 夹爪/转盘调参参考 |
| `VisionModule` | K230 扫任务二维码→HDMI | 语法过；未上板；待与主控 UART |
| `docs/ZDT_*` | 张大头闭环步进资料/例程 | Emm V5 帧格式权威 |
| `docs/PCA9685` | 舵机驱动板手册 | I2C PWM |
| `CubeMx/docs` | 大疆 C 板手册 + QGimbal | F407 硬件参考 |

## 2. 系统架构（目标）

```
手机/电脑蓝牙 9600 ──USART6──> carry_bot_v1 (F407 + FreeRTOS)
                                 ├─ app/Link     上位机链路（已有）
                                 ├─ app/Bridge   字符命令 → 业务（A-tier 已实现）
                                 ├─ app/ZDT      USART1 → Emm V5 电机总线（已实现）
                                 ├─ app/Servo    I2C1 → PCA9685 夹爪/转盘（已实现）
                                 └─ app/Motion   麦轮运动学（待建）
K230 VisionModule ──(另开 UART，勿占 USART1/6)──> 主控（待定）
```

地址约定（继承 USART 成功案例）：

| 地址 | 角色 |
|---|---|
| 1 | 升杆 |
| 2 | 转杆 |
| 3/4/5/6 | 麦轮 FL/RL/FR/RR |
| PCA9685 CH0/CH1 | 夹爪 / 物料转盘（`S` 前缀，非电机地址） |

## 3. Git 工作流（标准）

- **主线仓库**：[vesperadev/carry_bot_v1](https://github.com/vesperadev/carry_bot_v1)（账号 vesperadev；分支已推全）
- **`main`**：永远可编译基线
- **功能分支**（已从当前基线切出）：

| 分支 | 职责 | 建议负责人 |
|---|---|---|
| `feature/usart-module-port` | 总协调 / UART 回调统一 / 移植策略 | Mini_Agent |
| `feature/zdt-motor-bus` | Emm V5 组帧、USART1 DMA、地址槽 | 嵌入式 |
| `feature/bridge-bt-commands` | `M`/`S`/`X:` 命令解析，对接 Link+ZDT | Mini_Agent + 嵌入式 |
| `feature/servo-pca9685` | PCA9685 + 夹爪/转盘 | 嵌入式 |
| `feature/mecanum-kinematics` | 麦轮正/逆解与 `MFW` 等 | Mini_Agent / 嵌入式 |
| `docs/dev-handbook` | 本文档与进度 | Mini_Agent |

规则：

1. 一个分支只干一件事；完成一点 → 编译通过 → 提交 → 开 PR 进 `main`
2. **禁止**在 `USART` 参考工程里继续堆功能；只允许只读抄协议与命令语义
3. CubeMX 重新生成后必须核对 USART/I2C 的 DMA（USART 工程踩过坑）
4. 提交前缀：`feat:` / `fix:` / `refactor:` / `docs:`；不提交 `build/`
5. 合并前关闭参考工程的上电自检：`BOOTZERO_ENABLE` / `WHEEL_SELFTEST_ENABLE`

## 4. 移植路线（小步）

| # | 里程碑 | 验收 |
|---|---|---|
| 0 | Link BT 回显（已完成编译） | 烧录后蓝牙原样回显 |
| 1 | 统一 `HAL_UART_*` 回调按 Instance 分发 | BT 回显不回归 |
| 2 | `app/ZDT`：Emm 帧 + `ME1/MG/MQ/MX` | 单电机应答 `02 OK` |
| 3 | Bridge：`M?`/`ME`/`MG` 经蓝牙可用 | 与 `COMMANDS.md` 行为对齐（先子集） |
| 4 | 多地址槽 + 麦轮四轮 | `MFW`/`MSW` 实车 |
| 5 | PCA9685 `S` 命令 | 夹爪/转盘可控 |
| 6 | Vision UART 协议 | 任务码上屏且可回传主控 |

命令权威文档：`CubeMx/USART/COMMANDS.md`（完整）+ `USART/README.md`（接线/原理）。

## 5. 已知风险（必读）

- F103 参考用轮询+`HAL_Delay`；F407+RTOS 必须 DMA + 队列，NVIC 优先级 5–15
- 全工程只能有一份 `HAL_UARTEx_RxEventCallback` / `ErrorCallback`
- `Link_Send` 勿对同一缓冲重叠 `memcpy`（已记入 carry_bot 分析）
- 改电机 ID 时总线只能接一台；广播读类命令禁止
- 顶层目录含大量厂商资料，**不要**整目录塞进 git；文档用链接/子模块或 LFS

## 6. Bot 协作分工

| Bot | 职责 |
|---|---|
| **Mini_Agent**（本会话） | 总协调、分支策略、Bridge/文档、进度同步 |
| **嵌入式** | ZDT/USART1、PCA9685、HAL/CubeMX、烧录联调 |
| **Camelia** | 仓库整理、CI/文档润色、跨模块阻塞升级 |
| **3D** | 需要改机构/支架 STEP 时再拉（ZDT 装配模型在 `docs/`） |

协作约定：改代码前先在对应 feature 分支；阻塞写清「现象 / 已试 / 需要什么」。



## 7b. Bridge A-tier（`feature/bridge-bt-commands`）

`LINK_PEER=BT` 时，`app/Bridge` 作为 `LinkProtocol` 跑在 `linkTask` 内：按 `\r`/`\n` 切行，解析 `M*`/`S*`，调用 `Zdt_*` / `Servo_*`。

| 命令 | 行为 |
|------|------|
| `M?` | 帮助/探活 |
| `M[n]E0` / `M[n]E1` | 失能/使能（`n` 可省=缺省地址 2；`*`/`0`=广播） |
| `M[n]G[+|-]<deg>` | 按角度相对运动 |
| `M[n]X` | 立即停 |
| `MS[n]` / `M[n]S` | 设缺省地址 |
| `ML` | 列出 Bridge 侧槽位摘要 |
| `M[n]J` | 本地槽位状态（dir/vel/acc/last_deg） |
| `S?` / `SH` | 舵机帮助 |
| `S0U1000` / `S0U1500` | 夹爪关闭/打开（软件钳 1000–1500） |
| `S1U<us>` | 转盘绝对脉宽（硬件钳 200–2800） |
| `S0C` / `S0O` / `S1M` | 夹住 / 打开 / 转盘中位 |
| `test` / `TEST` | 轮 3–6 使能 + 同向速度跑约 2s 后停止/失能（无 kinematics 假设） |

B/C 档（`MV`/`MQ`/麦轮 `MFW`/`X:Y:`/摆动等）回 `ERR not implemented`。

冒烟（SerialTest / 蓝牙 9600）：

```text
M?
M1E1
M1G+10
M1X
S0U1500
S0U1000
S1U1500
test
```

本分支已 merge `feature/zdt-motor-bus` + `feature/servo-pca9685` 以便链接；上板验证后再合 main。

## 7. 本地构建（carry_bot_v1）

```powershell
$cmake = "C:\Users\37963\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe"
cd D:\GithubWork\Intelligent_Logistics_Handling\CubeMx\carry_bot_v1
& $cmake -S . -B build/Debug -DLINK_PEER=BT
& $cmake --build build/Debug --
```

---

变更记录：2026-11-09 初版（读完 docs / USART / carry_bot / Vision 后建立分支与协作）。- 2026-11-09 Bridge A-tier：`app/Bridge` + merge ZDT/Servo；BT 不再回显。

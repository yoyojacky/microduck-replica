# software · 软件适配

官方运行时 [`pollen-robotics/microduck`](https://github.com/pollen-robotics/microduck) 是照 Dynamixel XL330 写的，本仓库主线用飞特 HD-1910，软件要改。这个目录放的是**分析和方案**，实现的代码在 fork 出来的分支里。

## 代码在哪

> ### 👉 [`fanhao375/microduck` · `feetech` 分支](https://github.com/fanhao375/microduck/tree/feetech)
>
> 已跟上官方 **0.14.1**，三个 crate 的测试全过（duck-control 80 / robotd-params 151 / robotd 93）。**还没上过板**，等零位校准。

| 文件 | 行数 | 做了什么 |
|---|---|---|
| [`duck-control/src/bus_feetech.rs`](https://github.com/fanhao375/microduck/blob/feetech/duck-control/src/bus_feetech.rs) | 744（新） | **`FeetechIo` 主体**：每 tick 一次 `sync_read` 地址 56 长 15；位置换算带方向表；写目标带钳位；`set_gain` 只写 P（飞特没有 I/D 同构）；`reboot` 先关扭矩再 0x08；开机检查 END 字节序 / 应答级别 / 运行模式 / 波特率 / 相位；`imu_ready()` 恒 false。含 11 个单元测试 |
| [`duck-control/src/bus_select.rs`](https://github.com/fanhao375/microduck/blob/feetech/duck-control/src/bus_select.rs) | 149（新） | `AnyBus` 枚举，两种后端二选一（`Box<dyn RobotIo>` 编不过，原因见[架构文档](飞特适配架构.md) §3.1） |
| `robotd-params/` | +50 | `[bus] protocol = "dynamixel2" \| "feetech-sts"`、`[bus.feetech] p_scale / directions`，登记进参数表 |
| `robotd/src/main.rs` | +63 | `BusIo` 换成 `AnyBus`，开总线时带协议；参数不合法直接报错不重试 |
| `duck-control/src/model.rs` | +8 | HD-1910 出厂运行模式 4 的期望值 |
| `deploy/robotd.toml` | — | 两个新键，带注释 |

编译（需要 Rust）：

```bash
git clone -b feetech https://github.com/fanhao375/microduck.git
cd microduck && cargo test -p duck-control -p robotd-params -p robotd
```

**为什么放 fork 不放这个仓库**：`FeetechIo` 是官方 Rust 工作区里 `duck-control` 这个 crate 的一个文件，
用的是官方的 `RobotIo` trait、`Safety<T>` 泛型和参数系统 —— 单独拷出来编译不了。
放在 fork 分支上，官方更新时 `git rebase` 就能跟上（2026-09-20 跟了 55 个提交，零冲突），
将来也能直接向官方提 PR。官方是 Apache-2.0，改动留在 fork 里出处也清楚。

| 文件 | 内容 |
|---|---|
| [`飞特适配架构.md`](飞特适配架构.md) | 官方运行时现有架构（接缝在哪、每个 tick 干什么、开机序列）、两家舵机逐项差别、适配方案、**`imu_to_dxl` 小板固件架构（§4）**、台架验收、待查清单、构建部署 / 策略关系 / 失败模式、评审记录 |

小板固件的接口契约（15 字节块、时序、排队规则、验收）单独成文：[`hardware/imu_to_dxl/总线协议.md`](../hardware/imu_to_dxl/总线协议.md)，配套工具 [`tools/imu200/`](../tools/imu200/)（假小板 + 测试向量 + 真板子验收）。

板子怎么烧、怎么装官方运行时，在 [不打 HAT](../docs/不打HAT.md) 和 [`tools/radxa/`](../tools/radxa/)。

飞特协议和内存表的官方原文（2026 年版快照）在 [`docs/飞特资料/`](../docs/飞特资料/)。

动代码前先把舵机点通：[`tools/servo-web/`](../tools/servo-web/) 是接串口的网页调试台，拖滑块、看回读、导寄存器底账、3D 鸭子跟着动，架构文档台架清单 0–8 项都用它。

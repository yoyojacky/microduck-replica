# 上机分析脚本

2026-10-12 夜第一台上机时用来定位问题的脚本（10-13 夜更新了录像服务）。走路的步态分析在电脑上跑，见 [`tools/gait-analysis/`](../../gait-analysis/)。都是**只读**的：只订阅 robotd 的状态、读日志，不发任何控制指令（`webpad_probe.py` 例外，它只发「不按键、摇杆归零」的空帧）。
怎么用它们一步步找到「一接管就猛冲」的根因，见 [2026-10-12 夜上机记录](../../../docs/HD1910-20261012-夜-IMU校准与首次迈步.md)。

脚本假定的环境：主板上跑 kissqy 的 R17（robotd 套接字 `/run/robotd.sock`），录像放在 `/home/duck/r17/rec/`，脚本放在 `/home/duck/tools/`。

## 先把数据录下来

| 文件 | 在哪跑 | 干什么 |
|---|---|---|
| `duck_rec.py` | 主板 | 订阅 robotd 的 `robot.subscribe`，50 Hz 记关节角、目标、关节速度、IMU、安全状态、摇杆指令、策略状态，存 JSONL。`python3 duck_rec.py 秒数 输出文件`。追加写、**每秒写盘一次**；robotd 重启时记一行 `rec_error` 并自己重连 |
| `duck_health_log.py` | 主板 | 每 0.5 秒记一次 `robotctl health --json`：舵机电压、最热舵机、CPU 温度和频率、WiFi 和蓝牙状态，存 CSV |
| `duck-rec.service` | 主板 | 把上面两个做成开机自启，独立运行（不随 robotd 停），文件名带开机编号。放到 `/etc/systemd/system/`，`systemctl enable --now duck-rec` |
| `duck_journal.py`、`duck-journal.service` | 主板 | 把本次开机的系统日志实时抄到卡上，每秒写盘（Armbian 的日志在内存里，掉电就没了）。跳过 robotd 启动时 IMU 稳定阶段的刷屏 |

两个 service 文件里写死了用户 `duck` 和 `/home/duck/...` 路径（我们自己做的卡）；kissqy 一键镜像的用户是 `radxa`，用之前改成 `/home/radxa/...`。

录像约 3 MB/分钟。文件名是 `run-日期-时间-b开机编号前6位.jsonl`：主板没有 RTC，每次开机时钟都会回拨，**按开机编号找文件**（`cut -c1-6 /proc/sys/kernel/random/boot_id`）。每秒写盘，突然掉电最多丢最后 1 秒，坏行下面的脚本都会跳过。

## 出了问题怎么查

| 文件 | 回答什么问题 |
|---|---|
| `walk_now.py` | 最近一段模型接管（policy=walk）里，机身倾斜、角速度、腿目标偏离 HOME 多少、跟踪误差、摇杆指令 |
| `walk_report.py` | 最近 N 分钟推了几次摇杆，每次两条腿有没有交替迈步（左右膝角差的摆幅和换腿次数），倒没倒 |
| `failure_onset.py` | 从平稳到失控的转折点：腿目标第一次偏离 HOME 超过 15° 前后，逐帧看倾斜、重力、角速度、哪个关节先打满 |
| `imu_consistency.py` | IMU 陀螺仪和重力方向是不是自洽（dg/dt = −ω×g）。吻合度接近 1 说明轴向、正负号没错；接近 −1 或别的轴组合更吻合，就是配反了 |
| `vel_consistency.py` | 每个关节：角度差分（真实转速）和 robotd 报的速度比，查速度的正负号和比例 |
| `imu_fault_ctx.py` | IMU 每次断线时，机器人在干什么（静止 / 回 HOME / 走路）、电压多少，用来区分接触不良、供电跌落、驱动时序 |
| `webpad_probe.py` | 在主板本机测网页手柄那条路：webpad 冷启动多久、padd 能不能认到虚拟手柄 |
| `crash_scan.py` | 每段录像的状态切换时间线、摇杆指令、最后几帧的安全状态；带上同一段的 health CSV 就同时看电压、温度。用来判断「整机掉电」前一刻在干什么 |

## IMU 校准（今晚手动做的流程）

**第一步：方向检查**（第一次装 IMU、拆下来重装以后做）

鸭子扶竖直，往前、后、左（鸭子自己的左边）、右各倾斜约 20°，每个方向停 3 秒、回正。然后：

```
python3 imu_tilt_check.py 120        # 最近 120 秒，每 0.5 s 一行，偏离起点超过 8° 的标 ←
```

四段读数应该依次是「前倾 +」「前倾 −（后仰）」「右倾 −（左倾）」「右倾 +」，陀螺仪转动方向也要跟着对上。有一段符号反了，就是安装方向的轴配错了。

**第二步：零点校准**（摔狠了、碰过 IMU、换过线以后做）

模型训练时，鸭子站官方 HOME 姿势、脚平踩地面，**机身是水平的**（MJCF 的 `STAND` 关键帧，机身四元数是单位四元数，IMU 读 0°）。所以真机站 HOME，IMU 也应该读接近 0°；差多少，就是 IMU 模块装歪了多少。膝盖塌一点也没关系，下面按真实关节角算。

1. 鸭子放平地，按 Start 站到 HOME，手拿开，静止 3 秒；
2. 主板：`python3 capture_home.py home-capture.json`（取最后 100 帧的平均关节角和重力）；
3. 把 JSON 拷到装了 mujoco 和官方 `microduck_rl` 的电脑上：
   `python sim_home_tilt.py <microduck_rl>/src/mjlab_microduck/robot/microduck/scene_backlash.xml home-capture.json`
   输出「按真机关节角，机身应该前倾多少」和「IMU 安装偏差（实测 − 应有）」；
4. `python imu_mount_fix.py <前倾偏差> <右倾偏差> <当前安装四元数 w x y z>` 算出新的安装四元数；
5. 鸭子**卸力**以后，备份 `/etc/robot/feetech-ft5/hd1910-calibration.toml`，把 `["imu_mounts"."bno08x"]` 下的 `"quat"` 换成新值，`sudo systemctl restart robotd`；
6. 再站一次 HOME，重复 2～3，偏差应在 1° 以内。

2026-10-12 第一台：两种站姿算出的偏差是前倾 8.2° / 8.9°、右倾 1.7° / 1.8°，补 8.54° / 1.78° 后，复测剩 −0.55° / −0.47°。写进去的四元数是 `[0.454164155, 0.544101455, 0.527409117, 0.468538313]`（当时用两组数据做最小二乘，`imu_mount_fix.py 8.54 1.78` 算出的值与它差不到 0.01°）。

以后要把这套做成按钮式的手动校准工具（不自动改，每次校准都记日志），见上机记录的「下一步」。

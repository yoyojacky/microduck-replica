# 步态分析脚本（电脑上跑）

2026-10-13 夜～10-14 凌晨用来分析「走路发颤」和「蹭地」的脚本。全部**只读**：只读录像文件，不连鸭子。
怎么用它们一步步得出结论，见 [2026-10-13 夜上机记录](../../docs/HD1910-20261013-夜-掉电-舵机D与蹭地.md)。

**输入**：主板上 [`duck_rec.py`](../radxa/onboard-analysis/duck_rec.py) 录的 50 Hz JSONL（关节角、目标、IMU 重力和陀螺、摇杆指令、策略状态）。拷到电脑上再跑。
**环境**：Python 3.10+，`numpy`、`scipy`；带 `MJCF` 参数的脚本还要 `mujoco` 和官方 `microduck_rl` 仓库里的 `src/mjlab_microduck/robot/microduck/scene_backlash.xml`（正运动学用）。
`foot_clear.py`、`toe_decomp.py`、`stance_*.py`、`roll_pitch_coupling.py` **只统计「模型接管且推着摇杆」的帧**（`policy=walk` 且指令速度 > 0.02 m/s），站 HOME 的帧用来自检；`gait_quality.py` 自己分「站 HOME / 原地 / 在走」三段；`walk_transfer.py` 用所有 ≥3 秒的连续 `policy=walk` 段。录像里 robotd 重启过时，这几个脚本都会在那里断段。`home_roll.py` 用「保持 HOME」（`policy=held`、没卸力）的连续段，每段取最后 150 帧。

| 脚本 | 回答什么问题 | 用法 |
|---|---|---|
| `gait_quality.py` | 走得颤不颤、晃不晃：目标抖动和 >4 Hz 占比、跟踪误差和滞后、机身陀螺 RMS 和主频、步频 | `python gait_quality.py 录像.jsonl` |
| `walk_transfer.py` | 舵机跟不跟得上：各关节「目标 → 实际」的幅值比、滞后、相干（Welch 互谱），可以跟仿真录像并排比 | `python walk_transfer.py 名称=录像.jsonl ...` |
| `foot_clear.py` | 迈步脚离地多高：IMU 转正后，迈步脚最低点减撑地脚最低点；最高点时脚上最低的是脚尖还是脚跟；拆出「模型本意 / 迈步腿跟上 / 撑地腿不塌」 | `python foot_clear.py MJCF 名称=录像.jsonl[@起始wall[@结束wall]] ...` |
| `toe_decomp.py` | 迈步最高点脚尖朝下还是朝上，按关节和机身俯仰拆开 | `python toe_decomp.py MJCF 名称=录像.jsonl ...` |
| `stance_pitch.py` | 撑地时机身前倾多少，撑地腿的髋、膝、踝各自实际在哪、模型要它在哪 | 同上 |
| `stance_height.py` | 撑地腿把机身撑多高，比模型要的塌了多少 | 同上 |
| `home_roll.py` | 站 HOME 时两只脚掌左右方向差多少、是哪个关节造成的、躯干摆正时哪条边低（相对仿真 HOME；读数和目标同一个零点，零位偏差看不出来） | `python home_roll.py MJCF 名称=录像.jsonl ...` |
| `roll_pitch_coupling.py` | 横滚和俯仰是不是绑在一起：各频段都绑 = IMU 绕竖直轴装偏；只在步频附近绑 = 机身真的在点头 | `python roll_pitch_coupling.py 名称=录像.jsonl ...` |

**注意**：
- 带 `@` 的写法按录像里的 `wall`（Unix 时间）截取一段，一份录像里换过参数时用；
- `foot_clear.py` 等用 IMU 的重力方向把机身转正。IMU 零点偏 1°，两只脚在计算里就会差约 2 mm（脚间距约 98 mm）。先按 [IMU 校准](../radxa/onboard-analysis/README.md#imu-校准今晚手动做的流程) 把零点校好，再比较左右；站 HOME 那行「自检」应在 ±1 mm 以内；
- 仿真录像也能直接喂进来（字段一样），用来做对照。

# LSM6DSV16X 官方驱动

`lsm6dsv16x_reg.c`、`lsm6dsv16x_reg.h` 和 `LICENSE` 原样复制自 STMicroelectronics 官方仓库，固定提交 `2808e5cd6b85f91b66758e1dd0faab5f043aba07`。原始下载记录保存在 `source.json`；这份记录的路径是下载时的来源路径，不是构建依赖。

- 驱动来源：https://github.com/STMicroelectronics/lsm6dsv16x-pid/tree/2808e5cd6b85f91b66758e1dd0faab5f043aba07
- SFLP 配置参考：https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_sensor_fusion.c

板级 SPI 在 `Core/Src/board.c`，采样、FIFO 解析和恢复状态机在 `Core/Src/imu.c`。驱动版权和 BSD-3-Clause 许可保持不变。

本工程使用加速度 ±4g、角速度 ±500dps，加速度/陀螺仪/FIFO/SFLP 都为 120Hz。120Hz 是器件支持的标准速率，可为 50Hz 主控提供较新的姿态；不是把例程的 30Hz 直接套用。首次实板调试保留 ST 默认滤波链，没有加入未经测量的额外软件低通。

FIFO 的陀螺仪和四元数记录分别更新时间戳，不假设两个记录总是相邻。每轮最多取 16 条，1ms 轮询一次。每种必要记录超过 100ms 未更新就撤销 ready，1s 后自动重新配置；启动首批数据最多等待 1.5s。静止时数据值重复是正常现象，只有没有新的 FIFO 记录才算冻结。SPI 故障、FIFO 溢出、器件 ID 错误及软件复位超时都可诊断并重试。

四元数沿用 SFLP 的 x/y/z 半精度浮点，w 按官方例程和主控约定取非负平方根，不进行第二次安装姿态旋转。拒绝 NaN/Inf 和 x²+y²+z² > 1.02 的记录。允许半精度舍入导致的轻微超界，主控完成归一化。若一个已从 FIFO 读到的有效单位姿态恰好为三个正零，x 编码成 IEEE-754 负零 `0x8000`（数值仍然是零），避免主控把合法单位姿态误当成六字节全零的“未就绪”标记。上电未就绪、超时或故障时输出仍为全零。

本模块目前通过电脑上的寄存器/FIFO 模拟测试；尚未连接实板验证传感器配置、噪声、安装方向、时序或温漂。

`imu_get_snapshot()` 每次读取还会独立读取 `board_millis()` 检查样本年龄。即使繁忙的总线使采集轮询暂缓，超过 100ms 的数据也会返回 `ready=0`、`error=STALE` 和全零 gyro/quaternion；此时 `configured` 仍表示寄存器已经配置，诊断计数和时间戳不变，恢复操作由下一次 `imu_poll()` 执行。

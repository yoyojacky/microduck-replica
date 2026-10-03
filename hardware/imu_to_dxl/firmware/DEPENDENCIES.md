# 固件依赖与构建工具

编译所需的芯片与传感器驱动源码已保存在本目录。驱动无需在构建时联网下载；Keil、Arm Compiler、器件支持包和调试器工具需使用者自行合法安装。临时产物位于工程的 `Build` 目录。

| 组件 | 固定来源 | 许可证与本地改动 |
| --- | --- | --- |
| CMSIS Core 6.2.0 | [ARM-software/CMSIS_6 v6.2.0](https://github.com/ARM-software/CMSIS_6/tree/v6.2.0) | Apache-2.0；仅收录 Cortex-M0+ 与 Arm Compiler 6 所需头文件，内容未修改；许可证 `Drivers/CMSIS/LICENSE` |
| STM32G0 CMSIS Device v1.4.5 | [STMicroelectronics/cmsis-device-g0](https://github.com/STMicroelectronics/cmsis-device-g0/tree/f576c24e123edf3332988ecd49512c0f35f85186)，commit `f576c24e123edf3332988ecd49512c0f35f85186` | Apache-2.0；器件头与 system 源文件未修改；许可证 `Drivers/CMSIS/Device/ST/STM32G0xx/LICENSE.md` |
| STM32G031 启动代码 | 同一 ST commit 的 `Source/Templates/arm/startup_stm32g031xx.s` | 栈从1024改为2048字节；堆从512改为0字节。向量表、Reset_Handler 和弱 IRQ 名保留 ST 实现 |
| LSM6DSV16X PID 驱动 | [STMicroelectronics/lsm6dsv16x-pid](https://github.com/STMicroelectronics/lsm6dsv16x-pid/tree/2808e5cd6b85f91b66758e1dd0faab5f043aba07)，commit `2808e5cd6b85f91b66758e1dd0faab5f043aba07` | ST BSD-3-Clause；许可证、驱动原文件与来源记录在 `Drivers/LSM6DSV16X/` |
| Keil 器件支持包 | 验证版本 `Keil.STM32G0xx_DFP.2.1.0` | 提供设备描述、SVD 和 `CMSIS/Flash/STM32G0xx_64.FLM`；本工程另行收录官方 CMSIS 启动文件，避免 RTE 重复添加 |
| Arm Compiler for Embedded | 验证版本 6.24（0.1.0）、6.22（0.2.0，Keil MDK 5.41）；≥ 6.10 才能编 CMSIS 6 | 需有效工具授权；使用 Arm Compiler 6、旧语法 armasm 和 MicroLIB，仅隐藏旧汇编器弃用通知 A1950W |

## 构建入口

可用 VS Code 编辑源码，并在固件目录的 PowerShell 终端执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tools\build.ps1 -ToolchainRoot 'E:\Keil_v5\ARM\ARMCLANG'
```

将示例路径换成自己的 Arm Compiler 安装目录。仓库未附带 VS Code 任务配置。
构建会替换 `Build` 中的固件；仅使用姿态网页时应保留随工程提供的 `Build/imu_to_dxl.hex`，无需重新编译。
仓库只保留这一个预编译 HEX；`.bin/.map/.axf`、构建日志及测试产物均留在本地并由 `.gitignore` 排除。

Keil 可打开 `MDK-ARM/imu_to_dxl_jlink.uvprojx`（J-Link）或 `imu_to_dxl.uvprojx`（ST-LINK），首次使用核对自己的编译器、器件包、探针和烧录算法。J-Link 脚本将 SWD 速度设为已验证的100kHz。构建输出 `.axf/.hex/.bin/.map`；CLI脚本另生成构建日志和产物 SHA-256 清单。

两种入口使用 Cortex-M0+、C11、`-Oz`、软件浮点、短枚举、MicroLIB、`STM32G031xx` 和 `HSE_VALUE=16000000`。
链接布局：Flash `0x08000000..0x0800FFFF`，数据 RAM `0x20000000..0x200017FF`，最后2KiB `0x20001800..0x20001FFF` 为栈，初始 SP `0x20002000`。修改栈大小必须同时修改启动文件与 scatter 文件。

`tools/servo-web` 的 SWD / J-Link 观察器从 HEX 解码 10,724 字节镜像（0.2.0），使用 SHA-256
`66bc7c532f5a1ad87e3dfff71a4b51f8c4f20840a8f62436874fa5b7c665938e`
绑定已验证的 `sample=0x2000006C`（60字节）与 `tick=0x20000A34`（4字节），不依赖仓库提交 MAP。
重新编译后请用本地 MAP 复核符号及大小，并审核新的镜像哈希后再更新观察器。

## 主机测试与文件完整性

主机 C 测试需 MSVC；请先启动 **x64 Native Tools Command Prompt / Developer PowerShell**，再运行 `Tools/test.ps1 -PythonExe '自己的Python路径'`。测试入口不安装软件，也不连接硬件。

`Drivers/CMSIS/dependency-hashes.json` 记录收录驱动文件的 SHA-256；ST 源 commit 元数据见 `Drivers/CMSIS/Device/ST/STM32G0xx/source-commit.json`。构建和实板验证范围见 [VALIDATION.md](VALIDATION.md)。

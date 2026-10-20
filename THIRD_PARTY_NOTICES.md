# 第三方文件与许可范围

仓库根目录的 [MIT License](LICENSE) 适用于作者原创的应用代码、建模脚本、硬件设计、机械设计输出和项目文档。AI 辅助的使用过程记录在 [个人复盘](docs/校赛复盘与个人经验.md)。

仓库中随工程提供的第三方代码、库、符号、封装和参考素材，其原有版权与许可继续有效；根目录 MIT License 不替代这些文件的许可。

| 内容 | 位置与许可入口 |
| --- | --- |
| STM32F1 HAL | [Drivers/STM32F1xx_HAL_Driver/LICENSE.txt](f103_bringup/Drivers/STM32F1xx_HAL_Driver/LICENSE.txt)；同时保留源码内的版权头 |
| ARM CMSIS | [Drivers/CMSIS/LICENSE.txt](f103_bringup/Drivers/CMSIS/LICENSE.txt)；子目录内可能另附许可 |
| STM32F1 CMSIS 设备支持 | [Device/ST/STM32F1xx/LICENSE.txt](f103_bringup/Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt) |
| FreeRTOS | [Middlewares/Third_Party/FreeRTOS/Source/LICENSE](f103_bringup/Middlewares/Third_Party/FreeRTOS/Source/LICENSE) |
| three.js 三维预览库 | 各预览目录保留原文件与版权说明，例如 [V3 预览许可](pendulum_v3/output/preview/THREE-LICENSE.txt) |

第三方软件依赖由各自项目发布，安装依赖时遵循其许可证。硬件工程内引用的器件库以及参考图片、模块尺寸图等材料仍归原权利人；使用本项目原创设计时应保留这些来源信息。

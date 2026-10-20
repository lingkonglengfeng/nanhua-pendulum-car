# ATK-DMG474 四驱倒立摆接线方案

**当前选定方案 V6：两块 AT8236 双路模块**，每块驱动两台 12V 370 电机。主板只用 CN8/CN10 两根 24 芯 1:1 牛角座排线，IPS200 也经 CN10 连接。参阅 [V6 接线说明](output/AT8236双模块两排线方案_V6.md)、[元器件与线缆采购清单](output/AT8236转接板元器件与线缆采购清单_V6.md)、[SVG 关系图](output/08_AT8236双模块双牛角座_V6.svg)、[主板引脚表](output/pin_assignment_v6_at8236.csv) 和 [模块双排针表](output/AT8236_module_headers_v6.csv)。模块实际排距和方向须实测，SVG 是原理图参考，不是可投板封装。V5 TB6612 及以下版本均为历史方案。

**历史两排线方案 V4**：`output/双牛角座两排线方案_V4.md`、`output/06_双牛角座全接线_V4.svg`、`output/pin_assignment_v4_two_cables.csv`。V6 沿用其中四路编码器、I²C4 和 IPS200 的 SPI3 接法，但驱动器已换为 AT8236。

历史 DRV8871 实物照片中的 `VM/GND/IN1/IN2` 只适用于 V4；V6 的 AT8236 模块另外有 3.3V 逻辑 `VCC`，不可按旧模块针序插接。

**历史含调试屏的 V3**：`output/IPS200接入与引脚分配_V3.md`、`output/05_双排座加IPS关系图_V3.svg`、`output/pin_assignment_v3_ips.csv`。其中 IPS200 独走 CN3/SPI1；当前 V6 使用 CN10/SPI3。

> 座子脚号已勘误：此前 V2 中 PWM/方向/I²C 的 CN8/CN10 针号提前了两位。V6 引脚表使用已校正针号。完整 24 针对照见 `output/connector_pinout_verified.json`，依据官方原理图第 2 页和板面丝印第 6 页。此前截图中的旧针号请勿使用。

用户要求的 SVG 位于 `output/01_整车接线总图.svg` 和 `output/02_单路驱动与传感器细节.svg`，另附 PNG 预览、逻辑网络表和通道表。**新版实际引脚分配见 `output/DMG474引脚与转接板方案_V2.md` 和 `output/pin_assignment_v2.csv`；原 SVG/CSV 是 V1 功能网络图，里面的主控 `TBD` 不是最终管脚。**

图中主控根据用户最新照片识别为 **ATK-DMG474 / STM32G474VET6**，不是之前口头提及的 G407。该识别与 ST 的合作伙伴产品页一致。

资料盘中的 `DMG474_V1.0.pdf` 与 IO 分配表已用于新版 V2 分配，确认了 4 路 PWM、4 路方向、4 组硬件编码器及 I²C4 的 MCU/接口针号。投板前仍需核对接插件实物线序、各模块电气参数和 PC6/PC7 共用电路的 I²C 波形。

详见 [接线与转接板说明](output/接线与转接板说明.md)。SVG 可用浏览器、Inkscape 等打开；此包不是 PCB 打样文件，也未生成 Gerber。

重新绘图：Python 环境安装 `resvg-py` 后运行 `draw_wiring.py`。本机使用 `D:/blance_car/.cad-venv/Scripts/python.exe`。所有源图来自用户照片；未向外部上传。

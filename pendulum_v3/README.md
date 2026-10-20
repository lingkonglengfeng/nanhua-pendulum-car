# 车载单轴倒立摆 A1 mini V3 AS5600

V3 按用户提供的 Ø32 圆形 AS5600 模块尺寸图修改编码器背板。**只有 08 号打印件变化**；01–07、09、10 的 STL 与 V2 字节相同。已打印的 V1 底座也继续适用。

先看 [AS5600 安装说明](output/AS5600安装说明.md) 与 [完整采购和装配清单](output/采购与装配说明.md)。本次增加的采购件为 4 个长 6 mm、内径约 2.7、外径 6 的绝缘穿孔隔柱，4 套 M2.5×16 螺丝/螺母，以及适配模块的 GH1.25-4P 线。模块引脚顺序和供电接法不能从机械图推断。

`output/assembly.step` 为装配模型；`output/stl` 与 `output/3mf` 是打印文件，二选一使用。单独更新时只拿 `08_encoder_carrier`。`output/preview/assembly_viewer.html` 已改为独立离线预览，脚本、模型和备用装配图全部内嵌，可以单独复制后用 Edge/Chrome 打开。若 WebGL 无法启动会显示装配图及错误提示。旁边的 `three.min.js` 仅用于重新生成预览，打开成品 HTML 不再依赖它。

尺寸依据位于 `reference/AS5600_module_drawing.jpg`。PCB 与安装槽按图重建；接插件沿板面方向的深度、芯片平面包络为明确记录的建模假设。配套插头、线缆弯曲空间和实际磁铁仍需实物核对。保留 Ø6×3 直径方向充磁磁铁及原磁铁座，未假定卖家赠送磁铁一定相同。

修改 `parameters.json` 后执行：

```powershell
& 'D:/blance_car/.cad-venv/Scripts/python.exe' 'D:/blance_car/pendulum_v3/build_model.py'
& 'D:/blance_car/.cad-venv/Scripts/python.exe' 'D:/blance_car/pendulum_v3/check_exports.py'
& 'D:/blance_car/.cad-venv/Scripts/python.exe' 'D:/blance_car/pendulum_v3/verify_v3.py'
```

`verify_v3.py` 需要同级 V2 目录用于对照旧件。其他电脑安装 `requirements.txt` 后也可生成模型。STL/3MF 是推荐打印朝向，STEP 保留装配坐标；尺寸均为 mm。

已进行文件回读、几何干涉和旧件兼容性检查。本模型记录未包含实物试打、有限元、振动或控制性能验证。

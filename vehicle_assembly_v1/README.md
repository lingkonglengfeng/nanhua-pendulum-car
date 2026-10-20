# 倒立摆小车整车装配 V1

打开 `output/preview/assembly_viewer.html` 看整车，可切换底盘、摆轴/AS5600、配重细节，调整摆角和配重位置。HTML 内含模型、脚本及备用装配图，单独复制也可使用。图形加速无法启动时会显示装配图；推荐用 Edge/Chrome 打开。

`output/vehicle_complete_520.step` 是完整 CAD 装配，含 FR4 底板、V3 倒立摆、可调配重、轮组/电机参考、主要螺丝与电子模块布局。不是整件打印文件。尺寸单位 mm。分件加工和打印继续使用原各子项目的文件。

`output/preview/assembly.png` 为整车图；`three_views.png` 为正面、侧面、斜后方；`underside.png` 展示电池、绑带及轮组；`electronics.png` 展示板上电子模块。

详细说明见 [整车模型说明](output/整车模型说明.md)。零件表为 `output/component_manifest.csv`。

本机重建：用 `D:/blance_car/.cad-venv/Scripts/python.exe` 执行 `build_vehicle.py`。它只读取同级 `pendulum_v3`、`chassis_fr4_v1` 和 `pendulum_rod_weight` 的已生成 STEP/场景数据，输出到本目录。重新生成需要这些子项目，交付的 STEP 与 HTML 则可独立打开。

本装配为早期结构参考，未包含实物加工或控制性能验证记录。

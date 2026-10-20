# 倒立摆小车 FR4 底板 V1

新设计的完整车底板，尺寸 320×220×4 mm。现有 V3 两个倒立摆底座直接安装。520 L 型支架按用户提供的 30×24 四孔图直装；370 保留可调转接片与留白转接片，不把未提供的支架孔距当作已确认。

先看 [制造与装配说明](output/制造与装配说明.md)，加工厂使用 `output/dxf` 的毫米实尺寸文件，配合 `output/step` 的厚度。孔位功能图和坐标表也在 output 中。STL 仅供形状检查，主板不是可直接放入 A1 mini 的打印件。

`output/preview/assembly_viewer.html` 是独立离线预览，带装配图回退，可切换 520 直装与 370 转接示意。STEP 装配文件包含底盘和采购件参考；整车 HTML/PNG 另外加载了现有 V3 倒立摆。板卡、电池、轮毂和电机外形不是已核实的供应商模型。

本版底盘不修改原 V3 摆架文件。

## 再生成

本机使用 `D:/blance_car/.cad-venv/Scripts/python.exe` 运行 `build_chassis.py`。依赖 CadQuery 2.5.2、trimesh、numpy、matplotlib、ezdxf。完整预览需同级 `pendulum_v3/output`；`parameters.json` 记录设计参数，部分布局坐标在 `build_chassis.py` 的 holes 列表生成段中，不能认为改任意参数会自动同步所有孔阵。

运行 `check_geometry.py` 回读 DXF/STEP 并检查孔间材料宽度；运行 `check_viewer.cjs` 检查预览脚本和失败回退。没有实物加工、静载、振动或比赛控制性能验证。

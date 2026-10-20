# 车载单轴倒立摆 A1 mini V2

V2 为圆角改版，保留已经打印的 V1 `01_base_shoe` 底座和 `10_fit_coupon` 试打块，两者 STL 文件字节完全相同。下支撑臂的底部插接段也与 V1 几何完全相同；不用重打底座。后续 02–09 号采用这一版本，不混用两版编码器背板。

改动明细与验证结果见 [V2 圆角修改说明](output/V2圆角修改说明.md)。圆角处理改善几何过渡，但没有进行有限元分析或给出实测强度提升百分比。

这是可编辑、可分件打印的第一版上部机构。采用两个分段打印支撑臂、Ø8 金属轴、两只 608 轴承、单根铝管摆杆以及轴端磁编码器。

从 [采购与装配说明](output/采购与装配说明.md) 开始。`output/preview/assembly_viewer.html` 可以离线打开，需保留同目录的 `three.min.js`。装配 STEP 为 `output/assembly.step`；打印零件在 `output/stl/` 或 `output/3mf/`。

模型是几何样机，尚未实物试打或进行刚度、振动和控制性能验证。PCB 的外形和安装孔是明确标注的占位参数；购买具体模块后核对并修改。

## 修改与重新生成

修改 `parameters.json` 后执行 `build_model.py`，重新生成 STEP、STL、3MF、预览和验证报告。装配坐标：X 为前后，Y 为转轴方向，Z 向上，底盘安装面为 Z=0。输出零件 STL/3MF 已转换到推荐打印朝向；STEP 保留设计坐标。

本机执行：

```powershell
& 'D:/blance_car/.cad-venv/Scripts/python.exe' 'D:/blance_car/pendulum_v2/build_model.py'
```

其他电脑需 Python 3.12 与 `requirements.txt` 中的依赖。首次需将 Three.js 0.160.1 的 `three.min.js` 保留在 `output/preview/`；代码不依赖联网打开预览。

参数修改后必须查看新生成的 `verification.json`，不要将通过原始尺寸的检查等同于任意改型都通过。打印文件不是 G-code，3MF 不包含经验证的 Bambu 打印配置。

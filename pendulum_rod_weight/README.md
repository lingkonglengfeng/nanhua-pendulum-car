# 摆杆可调配重附件

给 V3 Ø8 铝管摆杆增加可移动配重，无需改动原模型。先读 [安装和调试说明](output/配重安装与调试说明.md)。

打印 `11_weight_clamp_head` 两件、`12_weight_clamp_nut` 两件。先各打一件试配。STL/3MF 二选一，3MF 没有预设打印机配置或 G-code。`weight_assembly.step` 为配重装配加原铝管参考，铝管不打印。

预览在 `output/preview/assembly_viewer.html`，为包含备用装配图的独立 HTML，可单独复制并用 Edge/Chrome 打开。滑块调整下夹环位置，默认固定 2 片垫片。另附已渲染 PNG。

生成：`python build_weight.py`。依赖 CadQuery 2.5.2、trimesh、numpy、matplotlib；完整小车预览与兼容性检查读取同级 `pendulum_v3/output/preview/scene.json`，不改写 V3。CAD 参数在 `parameters.json`；修改后需重跑验证。`check_viewer.cjs` 是 Node 下的脚本/降级逻辑检查，未验证实际浏览器 GPU。

此为打印试装件，本模型记录未包含实物夹紧力或控制性能验证。

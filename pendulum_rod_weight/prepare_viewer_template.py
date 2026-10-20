from pathlib import Path
r=Path(__file__).resolve().parent
s=(r.parent/'pendulum_v3/viewer_template.html').read_text(encoding='utf-8')
s=s.replace('倒立摆 A1 mini V3 — 3D 装配预览','摆杆可调配重 — 独立装配预览').replace('车载单轴倒立摆 · A1 mini V3','Ø8 摆杆可调配重')
s=s.replace('实际 CAD 网格预览 · 支撑臂分段打印 · 磁编码器固定在轴端','分体夹环 + M8 平垫片 · 可沿摆杆移动定位 · 预览采用 2 片垫片')
s=s.replace('轴端细节','配重细节').replace('<h2>装配与运动</h2>','<h2>配重位置与摆角</h2><label>下夹环下沿到摆轴 <strong id="positionText">160 mm</strong><input id="weightPosition" type="range" min="70" max="230" value="160" step="5"></label>')
s=s.replace('随轴转动的夹座 / 磁铁座','摆杆夹座 / 配重夹环').replace('采购金属件','采购金属件 / 配重垫片')
s=s.replace('编码器 PCB 为 30×30 mm、孔距 24×24 mm 的设计占位，选购后需核对。','图示两片 M8 平垫片，单片理论约 1.83 g；整套质量需实称。松开两夹环可移动，增加垫片可加重。此滑块是几何预览，不是控制效果仿真。')
(r/'viewer_template.html').write_text(s,encoding='utf-8')

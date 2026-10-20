from pathlib import Path
r=Path(__file__).resolve().parent
s=(r.parent/'pendulum_v3/viewer_template.html').read_text(encoding='utf-8')
s=s.replace('倒立摆 A1 mini V3 — 3D 装配预览','倒立摆 FR4 底盘 — 3D 装配预览').replace('车载单轴倒立摆 · A1 mini V3','FR4 底盘 · 520 直装 / 370 转接')
s=s.replace('实际 CAD 网格预览 · 支撑臂分段打印 · 磁编码器固定在轴端','主板 320×220×4 mm · 保留 V3 倒立摆 · 100 个安装孔与槽')
s=s.replace('轴端细节','底盘细节')
s=s.replace('<h2>装配与运动</h2>','<h2>装配与运动</h2><label>电机安装方式<select id="motorType"><option value="520">520：图纸支架直接固定</option><option value="370">370：可调转接片示意</option></select></label>')
s=s.replace('编码器 PCB 为 30×30 mm、孔距 24×24 mm 的设计占位，选购后需核对。','520 支架按用户 30×24 孔距图纸绘制。370 金属支架还需核对；可调转接片适用范围见说明。板卡与电池仅为布局包络。主底板为加工件，不能整件放进 A1 mini 打印。')
(r/'viewer_template.html').write_text(s,encoding='utf-8')

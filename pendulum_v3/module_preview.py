from pathlib import Path
import json,math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager
from make_preview import raster
root=Path(__file__).resolve().parent;out=root/'output'
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
d=json.loads((out/'preview/scene.json').read_text())
fig=plt.figure(figsize=(13,7),dpi=150,facecolor='#f5f7fa')
fig.text(.04,.95,'V3 · Ø32 AS5600 圆板安装',fontsize=21,weight='bold',color='#173447')
fig.text(.04,.895,'只更换 08 号背板，原滑轨与支架继续使用。',fontsize=12,color='#597080')
objs=[o for o in d if o['name'].startswith(('AS5600_','GH125_','NYLON_SPACER_')) or o['name'] in ['encoder_carrier','encoder_rail','magnet_cup','magnet_6x3']]
ax=fig.add_axes([.02,.20,.55,.64]);ax.imshow(raster(objs,950,760,22,-40));ax.axis('off')
ax=fig.add_axes([.60,.32,.36,.50])
pcb=next(o for o in d if o['name']=='AS5600_D32_PCB_FROM_DRAWING')
image_half_span=16*620/(620-24)
ax.imshow(raster([pcb],620,620,0,-90),extent=[-image_half_span,image_half_span,-image_half_span,image_half_span]);ax.set_aspect('equal')
# Overlay actual mount coordinates, whose projection scale is matched to the image bounds.
for x in [-8.5,8.5]:
    for z in [-8.5,8.5]:ax.add_patch(plt.Circle((x,z),1.0,fill=False,color='#ed9c45',lw=2))
ax.set(xlim=(-19,19),ylim=(-19,19));ax.axis('off');ax.set_title('四条斜槽固定 · 孔阵 17 × 17 mm',fontsize=12,color='#173447')
fig.text(.60,.23,'PCB Ø32 × 1.6 mm',fontsize=12,color='#173447')
fig.text(.60,.175,'4 个 6 mm 绝缘隔柱 + M2.5×16 螺丝',fontsize=12,color='#173447')
fig.text(.60,.12,'芯片面朝磁铁，插座面朝背板。',fontsize=12,color='#173447')
fig.text(.045,.08,'磁隙初始约 1.5 mm，可沿滑轨调整；插头与针序按卖家资料确认。',fontsize=11,color='#597080')
fig.savefig(out/'preview/AS5600_mount.png',facecolor=fig.get_facecolor());plt.close(fig)

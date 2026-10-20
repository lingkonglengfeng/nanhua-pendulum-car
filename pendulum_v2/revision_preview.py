from pathlib import Path
import json
import matplotlib.pyplot as plt
from matplotlib import font_manager
from make_preview import raster
root=Path(__file__).resolve().parent
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
old=json.loads((root.parent/'pendulum_v1/output/preview/scene.json').read_text())
new=json.loads((root/'output/preview/scene.json').read_text())
fig,axs=plt.subplots(2,2,figsize=(10,10),dpi=150,facecolor='#f5f7fa')
for row,name in enumerate(['lower_R','rod_hub']):
    for col,scene in enumerate([old,new]):
        selected=[o for o in scene if o['name']==name]
        axs[row,col].imshow(raster(selected,620,640 if row==0 else 400,0,-90))
        axs[row,col].axis('off')
        axs[row,col].set_title(('V1 原版' if col==0 else 'V2 圆角版')+' · '+('下段支撑臂' if row==0 else '摆杆夹座'),fontsize=13)
fig.suptitle('圆角修改对比 · 实际 CAD 正视图',fontsize=19,y=.98)
fig.text(.5,.04,'窗口内角 R4 / 夹座台阶 R2；底座插接段、轴孔与紧固孔位置保留。',ha='center',fontsize=11,color='#173447')
fig.tight_layout(rect=[0,.07,1,.95])
fig.savefig(root/'output/preview/fillet_comparison.png',facecolor=fig.get_facecolor())

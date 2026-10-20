import json,math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager
from render_support import raster

def render(root,out,scene,report):
    font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
    plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
    def art(objects,path,title,subtitle,elev,azim,w=1200,h=1100):
        fig=plt.figure(figsize=(12,10),dpi=150,facecolor='#f5f7fa')
        fig.text(.045,.945,title,fontsize=23,weight='bold',color='#173447')
        fig.text(.045,.899,subtitle,fontsize=11,color='#597080')
        ax=fig.add_axes([.025,.10,.95,.75]);ax.imshow(raster(objects,w,h,elev,azim));ax.axis('off')
        fig.text(.045,.047,'主控、电机外壳、轮毂与走线为布局参考；已设计零件沿用现有 CAD。',fontsize=10,color='#597080')
        fig.savefig(path,facecolor=fig.get_facecolor());plt.close(fig)
    art(scene,out/'preview/assembly.png','倒立摆小车 · 整车装配 V1','520 四驱 / 65 mm 轮 / 320×220 mm 底板 / AS5600 / 可调配重',19,-48,1000,1200)
    fig,axes=plt.subplots(1,3,figsize=(15,9),dpi=150,facecolor='#f5f7fa')
    fig.suptitle('整车三视图 · 零件外形参考',fontsize=21,y=.965)
    for ax,(name,e,a) in zip(axes,[('正面',0,0),('侧面',0,-90),('斜后方',18,135)]):
        ax.imshow(raster(scene,520,1000,e,a));ax.axis('off');ax.set_title(name,fontsize=14)
    fig.text(.5,.047,'参考外廓约 320 × 280 × 664 mm；高度随实际电机出轴偏置改变。',ha='center',fontsize=11,color='#597080')
    fig.tight_layout(rect=[0,.08,1,.92]);fig.savefig(out/'preview/three_views.png',facecolor=fig.get_facecolor());plt.close(fig)
    # Only lower parts to make underside and electronics visible.
    low=[o for o in scene if max(v[2] for v in o['v'])<55]
    art(low,out/'preview/underside.png','底盘底部与电池固定','板下 3S 电池占位 / 两条绑带 / 螺母与轮组 / 参考走线',-32,-50,1200,820)
    electronics=[o for o in low if not o['name'].startswith('base_')]
    art(electronics,out/'preview/electronics.png','电子模块与底盘布置','前端主控占位 / 后端四路 DRV8871 占位 / 中央 IMU / 电池在板下',50,-45,1200,850)
    from build_viewer import build
    build(root,out,scene,{'pivot_height_above_deck':320})

from pathlib import Path
import json
import matplotlib.pyplot as plt
from matplotlib import font_manager
from render_support import raster

def render(root,out,accessory,fullscene,p):
    font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
    plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
    fig=plt.figure(figsize=(13,8),dpi=150,facecolor='#f5f7fa')
    fig.text(.04,.945,'摆杆可调配重 · Ø8 mm 铝管适用',fontsize=23,weight='bold',color='#173447')
    fig.text(.04,.89,'分体夹环定位，金属平垫片增减重量；V3 主结构无需修改。',fontsize=12,color='#597080')
    ax=fig.add_axes([.01,.15,.42,.69]);ax.imshow(raster(fullscene,650,1000,16,-52));ax.axis('off')
    ax=fig.add_axes([.45,.40,.51,.43]);ax.imshow(raster(accessory,850,640,25,-58));ax.axis('off')
    fig.text(.49,.35,'上夹环',fontsize=13,color='#173447')
    fig.text(.49,.30,'中间：M8 平垫片，图示 2 片',fontsize=13,color='#173447')
    fig.text(.49,.25,'下夹环',fontsize=13,color='#173447')
    fig.text(.49,.17,'11 号 ×2 + 12 号 ×2；M2.5×16 螺丝 ×4',fontsize=12,color='#173447')
    fig.text(.04,.07,'位置以“下夹环下沿到摆轴”的距离计：70–230 mm。夹环和螺丝也要计入总配重。',fontsize=11,color='#597080')
    fig.savefig(out/'preview/assembly.png',facecolor=fig.get_facecolor());plt.close(fig)
    from build_viewer import build
    build(root,out,fullscene,{'pivot_height_above_deck':p['pivot_height']})

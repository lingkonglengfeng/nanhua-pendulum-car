import math,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle,FancyBboxPatch,Rectangle
from render_support import raster

COLORS={'BASE_M5':'#be722f','MOTOR_M4':'#7360ac','MOTOR520_M3':'#bf454a','BOARD_M3':'#27788a','IMU_M3':'#307856','SENSOR_M3':'#775393','BATTERY_STRAP':'#5c7080','WIRE':'#5c7080','SIDE_M3':'#5c7080'}
LABELS={'BASE_M5':'倒立摆底座：8 × Ø5.5','MOTOR520_M3':'520 直装：16 × Ø3.4','MOTOR_M4':'370/转接片：16 × 10×4.5 槽','BOARD_M3':'前后电子模块：36 × Ø3.4','IMU_M3':'中央 IMU：4 × Ø3.4','SENSOR_M3':'前后传感器：6 × 18×3.4 槽','BATTERY_STRAP':'板下电池：4 × 18×3.5 槽','WIRE':'布线：2 × 16×4 槽','SIDE_M3':'侧边扩展：8 × Ø3.4'}
def render(root,out,p,holes,chassis,full):
    font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc')
    plt.rcParams['font.family']=font_manager.FontProperties(fname='C:/Windows/Fonts/msyh.ttc').get_name()
    plt.rcParams['axes.unicode_minus']=False
    def draw_hole(ax,h):
        color=COLORS[h['group']]
        if h['kind']=='circle':ax.add_patch(Circle((h['x'],h['y']),h['diameter']/2,facecolor='white',edgecolor=color,lw=1))
        else:
            l,w=h['length'],h['width'];wx,wy=(l,w) if h['angle']==0 else (w,l)
            ax.add_patch(FancyBboxPatch((h['x']-wx/2,h['y']-wy/2),wx,wy,boxstyle=f'round,pad=0,rounding_size={w/2}',facecolor='white',edgecolor=color,lw=1))
    fig=plt.figure(figsize=(14,9),dpi=150,facecolor='white')
    fig.text(.045,.952,'倒立摆车底板 · FR4 320 × 220 × 4 mm',fontsize=22,weight='bold',color='#173447')
    fig.text(.045,.907,'顶视图 / 原点在板中心 / X 为前后、Y 为左右 / 所有尺寸为 mm',fontsize=11,color='#597080')
    ax=fig.add_axes([.045,.20,.70,.66]);ax.set_aspect('equal')
    ax.add_patch(FancyBboxPatch((-160,-110),320,220,boxstyle='round,pad=0,rounding_size=10',facecolor='#f4f6f0',edgecolor='#263b45',lw=1.6))
    for sy in [-1,1]:ax.add_patch(Rectangle((-80,sy*65-30),160,60,facecolor='#edb472',alpha=.16,edgecolor='#bd7c35',ls='--',lw=1.2))
    ax.add_patch(Rectangle((-80,-25),160,50,facecolor='#8ac8c6',alpha=.13))
    for h in holes:draw_hole(ax,h)
    ax.axhline(0,color='#bac5ca',lw=.6,ls=':');ax.axvline(0,color='#bac5ca',lw=.6,ls=':')
    ax.annotate('',xy=(160,-125),xytext=(-160,-125),arrowprops=dict(arrowstyle='<->',color='#263b45'));ax.text(0,-130,'320',ha='center',va='top',fontsize=11)
    ax.annotate('',xy=(-174,-110),xytext=(-174,110),arrowprops=dict(arrowstyle='<->',color='#263b45'));ax.text(-180,0,'220',ha='right',va='center',rotation=90,fontsize=11)
    ax.annotate('车头 +X',xy=(150,0),xytext=(95,0),ha='center',va='center',fontsize=10,color='#173447',arrowprops=dict(arrowstyle='->',color='#173447'))
    ax.text(-3,1,'O',ha='right',va='bottom',fontsize=10)
    ax.text(0,64,'现有底座 160×60',ha='center',fontsize=9,color='#9c672b')
    ax.text(0,-67,'现有底座 160×60',ha='center',fontsize=9,color='#9c672b')
    ax.text(0,39,'支撑臂中心距 130',ha='center',fontsize=9,color='#9c672b')
    ax.set(xlim=(-190,170),ylim=(-145,122));ax.axis('off')
    yy=.80
    for group in ['BASE_M5','MOTOR520_M3','MOTOR_M4','BOARD_M3','IMU_M3','SENSOR_M3','BATTERY_STRAP','WIRE','SIDE_M3']:
        fig.text(.77,yy,'●',color=COLORS[group],fontsize=13)
        fig.text(.795,yy,LABELS[group],fontsize=10,color='#173447');yy-=.054
    fig.text(.77,.255,'共 100 个孔 / 槽特征\n520 支架：按已提供图纸\n370 支架：可调接口待实物核对',fontsize=10,color='#597080',linespacing=1.8)
    fig.text(.045,.105,'中心通道器件高度 ≤15；前后示意电子模块总高 ≤28.6。电池布置在板下。',fontsize=11,color='#173447')
    fig.text(.045,.058,'加工用 DXF 为毫米实尺寸；此功能图用于识别孔位，不可按截图比例打孔。',fontsize=10,color='#597080')
    fig.savefig(out/'preview/chassis_top.png');fig.savefig(out/'preview/chassis_top.svg');plt.close(fig)

    fig=plt.figure(figsize=(14,9),dpi=150,facecolor='#f5f7fa')
    fig.text(.035,.95,'新底盘与原倒立摆装配',fontsize=23,weight='bold',color='#173447')
    fig.text(.035,.903,'520 支架直接安装 / 370 可更换转接片 / 65 mm 轮为占位外形',fontsize=11,color='#597080')
    visible=[o for o in full if not o['name'].startswith('ref_370')]
    ax=fig.add_axes([.015,.12,.44,.74]);ax.imshow(raster(visible,780,1120,18,-52));ax.axis('off')
    ax=fig.add_axes([.48,.40,.49,.43]);ax.imshow(raster([o for o in chassis if not o['name'].startswith('ref_370')],900,650,35,-48));ax.axis('off')
    fig.text(.51,.34,'前后通用电子模块孔阵，各 18 孔',fontsize=13,color='#173447')
    fig.text(.51,.285,'板下电池，中央摆动通道留空',fontsize=13,color='#173447')
    fig.text(.51,.23,'现有两个底座直接用 8 颗 M5 固定',fontsize=13,color='#173447')
    fig.text(.51,.16,'板卡、轮胎和电机外形仍需与实物核对。',fontsize=11,color='#597080')
    fig.text(.035,.065,'已校核孔位、实体与几何净空；未做实物刚度和控制性能试验。',fontsize=11,color='#597080')
    fig.savefig(out/'preview/assembly.png',facecolor=fig.get_facecolor());plt.close(fig)
    from build_viewer import build
    build(root,out,full,{'pivot_height_above_deck':320})

"""Render previews of the actual tessellated CAD; no generated concept art."""
from pathlib import Path
import json, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def raster(objects,w,h,elev,azim):
    """Orthographic rasterizer with per-pixel depth; avoids painter-sort artefacts."""
    e,a=np.radians([elev,azim]);d=np.array([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)])
    right=np.array([-np.sin(a),np.cos(a),0]);up=np.cross(d,right)
    basis=np.stack([right,up,d],axis=1)
    allv=np.concatenate([np.array(o['v'])[np.unique(np.array(o['f']).flatten())] for o in objects if len(o['f'])])
    coords=allv@basis;lo=coords[:,:2].min(0);hi=coords[:,:2].max(0)
    scale=min((w-24)/(hi[0]-lo[0]),(h-24)/(hi[1]-lo[1]))
    center=(lo+hi)/2
    image=np.full((h,w,3),[245,247,250],dtype=np.uint8);depth=np.full((h,w),-np.inf)
    light=np.array([.3,-.5,.812]);light/=np.linalg.norm(light)
    for o in objects:
        v=np.array(o['v']);f=np.array(o['f']);p=v@basis
        p[:,0]=(p[:,0]-center[0])*scale+w/2;p[:,1]=h/2-(p[:,1]-center[1])*scale
        color=np.array(matplotlib.colors.to_rgb(o['color']))*255
        for fi in f:
            q=p[fi];xmin=max(0,int(np.floor(q[:,0].min())));xmax=min(w-1,int(np.ceil(q[:,0].max())))
            ymin=max(0,int(np.floor(q[:,1].min())));ymax=min(h-1,int(np.ceil(q[:,1].max())))
            if xmax<xmin or ymax<ymin:continue
            (x0,y0,z0),(x1,y1,z1),(x2,y2,z2)=q
            den=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
            if abs(den)<1e-9:continue
            yy,xx=np.mgrid[ymin:ymax+1,xmin:xmax+1];xx=xx+.5;yy=yy+.5
            b0=((y1-y2)*(xx-x2)+(x2-x1)*(yy-y2))/den
            b1=((y2-y0)*(xx-x2)+(x0-x2)*(yy-y2))/den;b2=1-b0-b1
            zz=b0*z0+b1*z1+b2*z2
            region=depth[ymin:ymax+1,xmin:xmax+1]
            mask=(b0>=-1e-7)&(b1>=-1e-7)&(b2>=-1e-7)&(zz>region)
            if not mask.any():continue
            n=np.cross(v[fi[1]]-v[fi[0]],v[fi[2]]-v[fi[0]]);n/=max(np.linalg.norm(n),1e-9)
            brightness=.55+.45*abs(n@light)
            image[ymin:ymax+1,xmin:xmax+1][mask]=np.minimum(255,color*brightness).astype(np.uint8)
            region[mask]=zz[mask]
    return image

def render(root,out,scene,p):
    font=Path('C:/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams['axes.unicode_minus']=False
    H=p['pivot_height_above_deck']
    def paint(ax,objects,bounds,elev=19,azim=-53):
        all_polys=[];all_rgba=[]
        for o in objects:
            v=np.array(o['v']);f=np.array(o['f'])
            polys=v[f]
            color=matplotlib.colors.to_rgb(o['color'])
            n=np.cross(polys[:,1]-polys[:,0],polys[:,2]-polys[:,0])
            n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-10)
            lighting=0.58+0.42*np.abs(n@np.array([0.3,-0.5,0.812]))
            rgba=np.c_[np.clip(lighting[:,None]*color,0,1),np.ones(len(f))]
            all_polys.extend(polys);all_rgba.extend(rgba)
        pc=Poly3DCollection(all_polys,facecolors=all_rgba,edgecolors='none',zsort='average')
        ax.add_collection3d(pc)
        ax.set(xlim=bounds[0],ylim=bounds[1],zlim=bounds[2])
        ax.set_box_aspect([b[1]-b[0] for b in bounds]);ax.view_init(elev=elev,azim=azim)
        ax.set_axis_off()
    fig=plt.figure(figsize=(15,10),dpi=140,facecolor='#f5f7fa')
    fig.text(.045,.955,'车载单轴倒立摆 · A1 mini 打印版 V3',fontsize=23,weight='bold',color='#173447')
    fig.text(.045,.913,'基于实际 CAD 几何渲染  /  单位 mm  /  初版机械样机',fontsize=11,color='#597080')
    ax=fig.add_axes([.01,.10,.52,.78])
    ax.imshow(raster(scene,800,1150,19,-53));ax.axis('off')
    fig.text(.075,.10,'轴高 320  ·  有效摆长 280  ·  两支撑中心距 130',fontsize=11,color='#173447')
    fig.text(.075,.065,'灰色底板仅表示底盘安装范围；不包含电机、轮组。',fontsize=10,color='#597080')
    ax2=fig.add_axes([.48,.36,.51,.43])
    close=[o for o in scene if o['name'] not in ['CHASSIS_DECK_ENVELOPE_ONLY','tube_8x6x260'] and not any(a in o['name'] for a in ['base_','lower_','splice_','M4_tie'])]
    # Crop upper arms visually by filtering triangles, while preserving actual mesh.
    clipped=[]
    for o in close:
        a=dict(o);v=np.array(o['v']);f=np.array(o['f'])
        a['f']=f[np.all(v[f][:,:,2]>H-35,axis=1)].tolist()
        if a['f']:clipped.append(a)
    ax2.imshow(raster(clipped,960,620,25,-43));ax2.axis('off')
    fig.text(.565,.825,'轴端与编码器结构',fontsize=15,weight='bold',color='#173447')
    fig.text(.565,.335,'橙色：摆杆夹座 / 旋转磁铁座',fontsize=12,color='#7e511b')
    fig.text(.565,.294,'紫色：固定滑轨 / 编码器背板',fontsize=12,color='#70508e')
    fig.text(.565,.253,'绿色：按用户图纸建模的 Ø32 AS5600 模块',fontsize=12,color='#386848')
    fig.text(.565,.20,'磁铁随轴旋转，传感器不承重。',fontsize=12,color='#173447')
    fig.text(.565,.15,'已检查实体与打印包络；尚未实物试打或验证刚度。',fontsize=10,color='#597080')
    fig.savefig(out/'preview'/'assembly.png',facecolor=fig.get_facecolor());plt.close(fig)

    # Orthographic dimensioned design sheet (outline, not manufacturing drawing).
    fig,axs=plt.subplots(1,2,figsize=(13,8),dpi=150,facecolor='white')
    fig.suptitle('安装尺寸与摆动空间（mm）',fontsize=19,y=.98)
    a=axs[0]
    for s in [-1,1]:
        a.add_patch(plt.Rectangle((s*65-6,8),12,336,facecolor='#198b9a',alpha=.65))
        a.add_patch(plt.Rectangle((s*65-30,0),60,8,facecolor='#395269'))
    a.plot([-90,100],[H,H],color='#899ba7',lw=5)
    a.plot([0,0],[H+20,H+280],color='#ed9c45',lw=5)
    a.plot([-120,120],[0,0],color='#899ba7',lw=3)
    a.annotate('',xy=(-65,-24),xytext=(65,-24),arrowprops=dict(arrowstyle='<->',color='#173447'))
    a.text(0,-39,'130 支撑中心距',ha='center',fontsize=10)
    a.annotate('',xy=(145,0),xytext=(145,H),arrowprops=dict(arrowstyle='<->',color='#173447'))
    a.text(151,H/2,'320 轴高',rotation=90,va='center',fontsize=10)
    a.set_title('正视图：沿车头方向看',fontsize=12)
    a.set(xlim=(-145,185),ylim=(-55,625));a.set_aspect('equal');a.axis('off')
    a=axs[1]
    circle=plt.Circle((0,H),280,fill=False,color='#d69b50',lw=1.5,ls='--')
    a.add_patch(circle)
    a.plot([0,0],[H,H+280],color='#ed9c45',lw=4)
    a.plot([0,0],[H,H-280],color='#ed9c45',lw=2,ls='--')
    a.plot([-150,150],[0,0],color='#899ba7',lw=3)
    a.scatter([0],[H],s=35,c='#173447')
    a.annotate('',xy=(36,0),xytext=(36,40),arrowprops=dict(arrowstyle='<->',color='#173447'))
    a.text(51,22,'约 40 净空',va='center',fontsize=10)
    a.text(0,210,'虚线内为摆动扫掠区\n中心平面不可布置横梁 / 电池',ha='center',fontsize=11,color='#7e511b')
    a.text(0,-36,'底板以下：另购小车底盘',ha='center',fontsize=10)
    a.set_title('侧视图：摆杆绕横向轴旋转',fontsize=12)
    a.set(xlim=(-305,305),ylim=(-55,625));a.set_aspect('equal');a.axis('off')
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(out/'preview'/'dimensions.png');plt.close(fig)

    from build_viewer import build
    build(root,out,scene,p)
    print('PREVIEWS_WRITTEN',flush=True)

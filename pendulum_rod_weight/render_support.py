import numpy as np
import matplotlib
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


"""Independent hole-ligament calculation and export readback."""
from pathlib import Path
import json,math
import numpy as np
import cadquery as cq
import ezdxf
root=Path(__file__).resolve().parent;out=root/'output'
holes=json.loads((out/'hole_features.json').read_text(encoding='utf-8'))
def segment(h):
    c=np.array([h['x'],h['y']],float)
    if h['kind']=='circle':return c,c,h['diameter']/2
    a=(h['length']-h['width'])/2;t=math.radians(h['angle']);v=a*np.array([math.cos(t),math.sin(t)])
    return c-v,c+v,h['width']/2
def pointseg(p,a,b):
    ab=b-a;n=ab@ab
    return np.linalg.norm(p-(a+np.clip((p-a)@ab/n,0,1)*ab)) if n>1e-10 else np.linalg.norm(p-a)
def cross(a,b):return a[0]*b[1]-a[1]*b[0]
def distance(a,b,c,d):
    u=b-a;v=d-c;den=cross(u,v)
    if abs(den)>1e-9:
        t=cross(c-a,v)/den;s=cross(c-a,u)/den
        if 0<=t<=1 and 0<=s<=1:return 0
    return min(pointseg(a,c,d),pointseg(b,c,d),pointseg(c,a,b),pointseg(d,a,b))
minimum=(1e9,None)
for i,h in enumerate(holes):
    a,b,r=segment(h)
    for k in holes[i+1:]:
        c,d,s=segment(k);gap=float(distance(a,b,c,d)-r-s)
        if gap<minimum[0]:minimum=(gap,[h['id'],k['id']])
assert minimum[0]>=5.0,minimum
main=cq.importers.importStep(str(out/'step/01_chassis_FR4_320x220x4.step')).val()
assert main.isValid() and len(main.Solids())==1
assert len(holes)==100
for f in (out/'dxf').glob('*.dxf'):
    doc=ezdxf.readfile(f);assert doc.units==4;assert not doc.audit().has_errors
result={'minimum_hole_to_hole_ligament_mm':round(minimum[0],3),'closest_pair':minimum[1],'main_hole_slot_count':100,'step_valid':True,'dxf_units_mm':True,'dxf_audit_passed':True}
(out/'export_readback.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))

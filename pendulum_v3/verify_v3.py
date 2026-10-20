from pathlib import Path
import json,hashlib
import cadquery as cq
root=Path(__file__).resolve().parent;old=root.parent/'pendulum_v2/output';out=root/'output'
unchanged=[]
for path in sorted((out/'stl').glob('*.stl')):
    if path.name.startswith('08_'):continue
    assert hashlib.sha256(path.read_bytes()).digest()==hashlib.sha256((old/'stl'/path.name).read_bytes()).digest(),path.name
    unchanged.append(path.name)
assert len(unchanged)==9
assert (out/'stl/08_encoder_carrier.stl').read_bytes()!=(old/'stl/08_encoder_carrier.stl').read_bytes()
p=json.loads((root/'parameters.json').read_text(encoding='utf-8'))
checks=json.loads((out/'verification.json').read_text(encoding='utf-8'))['checks']
yl,yh=checks['encoder_carrier_rail_hole_y_range_mm'];y=checks['as5600_rail_fastener_y_mm']
assert yl<y<yh
back=cq.importers.importStep(str(out/'step/08_encoder_carrier.step')).val()
def cy(r,x,y,z,length):return cq.Solid.makeCylinder(r,length,cq.Vector(x,y,z),cq.Vector(0,1,0))
center=checks['as5600_backplate_center_y_mm'];H=p['pivot_height_above_deck']
for x in [-8.5,8.5]:
    for z in [H-8.5,H+8.5]:assert back.intersect(cy(1.4,x,center-2,z,4)).Volume()<1e-6
result={'reference_drawing':'reference/AS5600_module_drawing.jpg','unchanged_stl_byte_identical_to_v2':unchanged,'changed_printed_part':'08_encoder_carrier','mount_pitch_mm':[17,17],'slider_fastener_y_mm':y,'within_original_rail':True,'needs_physical_trial':True}
(out/'revision_compatibility.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))

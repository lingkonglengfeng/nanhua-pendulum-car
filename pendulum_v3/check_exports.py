"""Independent read-back of deliverables, including STEP and 3MF containers."""
from pathlib import Path
import json, zipfile, xml.etree.ElementTree as ET
import cadquery as cq
import numpy as np

root=Path(__file__).resolve().parent
out=root/'output'
step=cq.importers.importStep(str(out/'assembly.step'))
assert step.val().isValid()
solids=len(step.solids().vals())
assert solids==36
ns={'m':'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
checked=[]
for path in sorted((out/'3mf').glob('*.3mf')):
    with zipfile.ZipFile(path) as z:
        assert z.testzip() is None
        xml=ET.fromstring(z.read('3D/3dmodel.model'))
        assert xml.attrib['unit']=='millimeter'
        vertices=xml.findall('.//m:vertex',ns)
        triangles=xml.findall('.//m:triangle',ns)
        xyz=np.array([[float(v.attrib[a]) for a in ('x','y','z')] for v in vertices])
        assert len(vertices)>0 and len(triangles)>0
        assert xyz.min()>=-0.001 and np.max(np.ptp(xyz,axis=0))<180
        assert max(int(t.attrib[a]) for t in triangles for a in ('v1','v2','v3'))<len(vertices)
        checked.append(path.name)
assert len(checked)==10
result={'assembly_step_valid_after_reimport':True,'assembly_solid_count':solids,
        'geometry_3mf_readback_pass':checked,'preview_note':'V3 AS5600 D32 module reconstructed from user drawing, including connector envelopes and four spacers.',
        'not_performed':['physical print','Bambu Studio slicing','load or stiffness test','control-system test']}
(out/'export_readback.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))

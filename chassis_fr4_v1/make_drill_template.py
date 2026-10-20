"""Real-size vector sheet for A3 100% printing; CNC must use DXF instead."""
from pathlib import Path
import json,xml.etree.ElementTree as ET
r=Path(__file__).resolve().parent;o=r/'output'
h=json.loads((o/'hole_features.json').read_text(encoding='utf-8'))
parts=['<svg xmlns="http://www.w3.org/2000/svg" width="340mm" height="255mm" viewBox="0 0 340 255">',
       '<rect width="340" height="255" fill="white"/>',
       '<text x="170" y="6" text-anchor="middle" font-size="3.5" font-family="sans-serif">CHASSIS 320 x 220 / mm / A3 LANDSCAPE / PRINT 100%</text>',
       '<rect x="10" y="10" width="320" height="220" rx="10" fill="none" stroke="black" stroke-width="0.25"/>']
for f in h:
    x,y=170+f['x'],120-f['y']
    if f['kind']=='circle':parts.append(f'<circle cx="{x}" cy="{y}" r="{f["diameter"]/2}" fill="none" stroke="black" stroke-width="0.18"/>')
    else:
        l,w=f['length'],f['width'];a=-f['angle']
        parts.append(f'<rect x="{-l/2}" y="{-w/2}" width="{l}" height="{w}" rx="{w/2}" transform="translate({x},{y}) rotate({a})" fill="none" stroke="black" stroke-width="0.18"/>')
    parts.append(f'<path d="M{x-1},{y}h2 M{x},{y-1}v2" stroke="#777" stroke-width="0.12"/>')
parts.extend(['<path d="M20 245h100 M20 242v6 M120 242v6" fill="none" stroke="black" stroke-width="0.3"/>',
              '<text x="70" y="252" text-anchor="middle" font-size="3" font-family="sans-serif">100 mm - measure before drilling</text>',
              '<text x="200" y="246" font-size="3" font-family="sans-serif">Do not use fit-to-page or screenshot scaling.</text>','</svg>'])
path=o/'drill_template_1to1_A3.svg';path.write_text('\n'.join(parts),encoding='utf-8');ET.parse(path)
print(path)

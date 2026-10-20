"""Build a standalone serial monitor using only files in this directory."""
from pathlib import Path
import json

root = Path(__file__).resolve().parent
capture = root / 'samples' / 'swing_trace_v39.log'
seed = {'name': '实际起摆与直立日志 · v39 · 2026-10-18', 'text': capture.read_text(encoding='utf-8-sig')}
source = (root / 'monitor.template.html').read_text(encoding='utf-8')
packed = json.dumps(seed, ensure_ascii=False).replace('<', '\\u003c')
for marker, content in {
    '__EMBEDDED_LOG_JSON__': packed,
    '__PROTOCOL_CORE__': (root / 'protocol-core.js').read_text(encoding='utf-8'),
    '__MONITOR_APP__': (root / 'monitor-app.js').read_text(encoding='utf-8'),
}.items():
    assert source.count(marker) == 1, marker
    source = source.replace(marker, content)
target = root / 'as5600-monitor.html'
target.write_text(source, encoding='utf-8')
print(f'Built {target} ({target.stat().st_size} bytes)')

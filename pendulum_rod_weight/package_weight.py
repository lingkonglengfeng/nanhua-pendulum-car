from pathlib import Path
import zipfile
r=Path(__file__).resolve().parent;t=r.parent/'pendulum_摆杆可调配重.zip'
with zipfile.ZipFile(t,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in r.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:z.write(p,Path(r.name)/p.relative_to(r))
with zipfile.ZipFile(t) as z:
    assert z.testzip() is None
    print(str(t),len(z.namelist()),round(t.stat().st_size/1048576,2),'MiB')

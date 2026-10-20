from pathlib import Path
import zipfile
r=Path(__file__).resolve().parent;target=r.parent/'pendulum_整车装配_V1.zip'
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in r.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:z.write(p,Path(r.name)/p.relative_to(r))
with zipfile.ZipFile(target) as z:
    assert z.testzip() is None
    print(str(target),len(z.namelist()),round(target.stat().st_size/1048576,2),'MiB')

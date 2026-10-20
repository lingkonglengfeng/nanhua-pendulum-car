from pathlib import Path
import zipfile
root=Path(__file__).resolve().parent
target=root.parent/'pendulum_A1mini_V2.zip'
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc':
            z.write(p,Path(root.name)/p.relative_to(root))
with zipfile.ZipFile(target) as z:
    assert z.testzip() is None
    print(f'{target}: {len(z.namelist())} files, {target.stat().st_size/1024/1024:.2f} MiB')

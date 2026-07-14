from stl import mesh
from pathlib import Path
_repo_root = next(
    p for p in Path(__file__).resolve().parents
    if (p / 'data').is_dir() and (p / 'pyrpod').is_dir()
)
_DATA_STL = _repo_root / 'data' / 'stl'
import math

mesh = mesh.Mesh.from_file(str(_DATA_STL / 'mold_funnel_high_res.stl'))

mesh.translate([-0.217368/2, -1.6/2, -1.6/2])

mesh.save('mold_funnel_high_res_transformed.stl')

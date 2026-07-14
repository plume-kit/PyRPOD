from stl import mesh
from pathlib import Path
_repo_root = next(
    p for p in Path(__file__).resolve().parents
    if (p / 'data').is_dir() and (p / 'pyrpod').is_dir()
)
_DATA_STL = _repo_root / 'data' / 'stl'
import math

mesh = mesh.Mesh.from_file(str(_DATA_STL / 'convex_tv.stl'))

# convex_tv
mesh.translate([0,-56.5331761/2,-56.5331761/2])

mesh.save('convex_tv_transformed.stl')
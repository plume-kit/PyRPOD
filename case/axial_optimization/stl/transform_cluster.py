from stl import mesh
from pathlib import Path
_repo_root = next(
    p for p in Path(__file__).resolve().parents
    if (p / 'data').is_dir() and (p / 'pyrpod').is_dir()
)
_DATA_STL = _repo_root / 'data' / 'stl'
import math

mesh = mesh.Mesh.from_file(str(_DATA_STL / 'cluster.stl'))

mesh.translate([-0.306/2,-0.325/2,-0.3799/2]) # center on origin

mesh.translate([-0.306/2,0,0]) # line up edge of tc with edge of lm

mesh.save('cluster_transformed.stl')
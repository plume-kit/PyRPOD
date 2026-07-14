from stl import mesh
from pathlib import Path
_repo_root = next(
    p for p in Path(__file__).resolve().parents
    if (p / 'data').is_dir() and (p / 'pyrpod').is_dir()
)
_DATA_STL = _repo_root / 'data' / 'stl'
import math

mesh = mesh.Mesh.from_file(str(_DATA_STL / 'lm.stl'))

# center radially on x-axis and align face of LM's docking port to plane at x = 0
mesh.translate([-9.605,-9.659079/2,-9.659079/2])

# align edge of LM's outer diameter on the docking side to plane at x = 0
mesh.translate([0.5,0,0])

mesh.save('lm_transformed.stl')
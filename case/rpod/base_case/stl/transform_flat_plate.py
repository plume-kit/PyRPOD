from stl import mesh
from pathlib import Path
_repo_root = next(
    p for p in Path(__file__).resolve().parents
    if (p / 'data').is_dir() and (p / 'pyrpod').is_dir()
)
_DATA_STL = _repo_root / 'data' / 'stl'
import math

mesh = mesh.Mesh.from_file(str(_DATA_STL / 'flat_plate_low_res.stl'))

mesh.translate([-1/2, -70/2, -10/2])

mesh.save('flat_plate_low_res_transformed.stl')
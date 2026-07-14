from stl import mesh
from pathlib import Path
_repo_root = next(
    p for p in Path(__file__).resolve().parents
    if (p / 'data').is_dir() and (p / 'pyrpod').is_dir()
)
_DATA_STL = _repo_root / 'data' / 'stl'
import numpy as np
import math

# for example
cant_angle = 0

mesh = mesh.Mesh.from_file(str(_DATA_STL / 'thruster_ATV216.stl'))

# center on origin
mesh.translate([-0.19/2,-0.095/2,-0.095/2])

# SweepConfig needs to manipulate the dcm and the thruster should be rotated before the following translation
# center point of thruster that intersects with cluster on origin
mesh.translate([0.095*np.cos(math.radians(cant_angle)),0,0.095*np.sin(math.radians(cant_angle))])

mesh.save('thruster_ATV216_transformed.stl')
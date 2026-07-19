import numpy as np
from stl import mesh

# Define the dimensions and divisions of the flat plate
plate_width = 10.0
plate_length = 10.0
plate_thickness = 0.5
division_width = 3  # Divisions in width
division_length = 3  # Divisions in length

# Calculate increments for divisions
increment_width = plate_width / (division_width + 1)
increment_length = plate_length / (division_length + 1)

# Create the vertices of the plate
vertices = []
for i in range(division_width + 2):
    for j in range(division_length + 2):
        vertices.append([i * increment_width, j * increment_length, 0])
        vertices.append([i * increment_width, j * increment_length, plate_thickness])

vertices = np.array(vertices)

# Define the triangles using vertices indices
faces = []
for i in range(division_width + 1):
    for j in range(division_length + 1):
        v0 = i * (division_length + 2) + j
        v1 = v0 + 1
        v2 = v0 + division_length + 3
        v3 = v0 + division_length + 2

        faces.append([v0, v1, v2])
        faces.append([v0, v2, v3])

faces = np.array(faces)

# Create the mesh
flat_plate = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
for i, face in enumerate(faces):
    for j in range(3):
        flat_plate.vectors[i][j] = vertices[face[j], :]

# Ensure outward-facing normals
flat_plate.normals[:] = np.cross(
    flat_plate.vectors[:, 1] - flat_plate.vectors[:, 0],
    flat_plate.vectors[:, 2] - flat_plate.vectors[:, 0]
)

# Save the mesh to an STL file
flat_plate.save('flat_plate_with_divisions.stl')
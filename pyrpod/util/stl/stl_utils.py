import argparse
import json
import os
from stl import mesh
import numpy as np

def load_stl(file_path):
    """
    Load an STL file and return a mesh object.

    Parameters
    ----------
    file_path : str
        Path to the STL file.

    Returns
    -------
    mesh.Mesh
        The loaded STL mesh object.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"STL file not found: {file_path}")
    return mesh.Mesh.from_file(file_path)

def transform_mesh(mesh_obj, rotation_matrix=None, translation_vector=None, scale_factor=None):
    """
    Apply transformations to a mesh object.

    Parameters
    ----------
    mesh_obj : mesh.Mesh
        The mesh object to transform.
    rotation_matrix : np.ndarray, optional
        A 3x3 rotation matrix to apply to the mesh.
    translation_vector : list or np.ndarray, optional
        A 3-element vector to translate the mesh.
    scale_factor : float, optional
        A scaling factor to apply to the mesh.

    Returns
    -------
    mesh.Mesh
        The transformed mesh object.
    """
    if scale_factor:
        mesh_obj.points *= scale_factor
    if rotation_matrix is not None:
        mesh_obj.rotate_using_matrix(rotation_matrix)
    if translation_vector is not None:
        mesh_obj.translate(translation_vector)
    return mesh_obj

def transform_mesh_from_file(input_file, output_file, scale, translate):
    """
    Transform an STL mesh with scaling and translation and save it to a file.

    Parameters
    ----------
    input_file : str
        Path to the input STL file.
    output_file : str
        Path to the output STL file.
    scale : float
        Scaling factor to apply to the mesh.
    translate : list or np.ndarray
        Translation vector to apply to the mesh.
    """
    # Load the mesh from the file
    model = load_stl(input_file)

    # Apply transformations
    model = transform_mesh(model, scale_factor=scale, translation_vector=translate)

    # Save the transformed mesh to the output file
    model.save(output_file)
    print(f"Mesh saved to {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Transform an STL mesh with scaling and translation using a configuration file.")

    # Configuration file
    parser.add_argument("config_file", type=str, help="Path to the configuration JSON file.")

    args = parser.parse_args()

    # Load configuration from the file
    with open(args.config_file, 'r') as config_file:
        config = json.load(config_file)

    input_file = config.get("input_file")
    output_file = config.get("output_file")
    scale = config.get("scale", 1.0)
    translate = config.get("translate", [0.0, 0.0, 0.0])

    # Transform the mesh using the configuration
    transform_mesh_from_file(
        input_file=input_file,
        output_file=output_file,
        scale=scale,
        translate=translate
    )
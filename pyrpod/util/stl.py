from stl import mesh
import os

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
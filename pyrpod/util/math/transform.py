import numpy as np


def rotation_matrix_from_vectors(vec1: np.ndarray, vec2: np.ndarray) -> np.ndarray:
    """
    Compute the 3x3 rotation matrix that rotates vec1 to align with vec2 using
    the Rodrigues' rotation formula.

    Parameters
    ----------
    vec1 : array_like, shape (3,)
        Source vector.
    vec2 : array_like, shape (3,)
        Destination vector.

    Returns
    -------
    rotation_matrix : ndarray, shape (3,3)
        Rotation matrix which when applied to vec1 aligns it to vec2.
    """
    vec1 = np.asarray(vec1, dtype=float).reshape(3)
    vec2 = np.asarray(vec2, dtype=float).reshape(3)

    # Handle identical vectors quickly
    if np.allclose(vec1, vec2):
        return np.eye(3)

    a = vec1 / np.linalg.norm(vec1)
    b = vec2 / np.linalg.norm(vec2)

    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)

    # If vectors are opposite, choose an orthogonal axis for 180-degree rotation
    if np.isclose(s, 0) and c < 0:
        # Find a vector orthogonal to a
        ortho = np.array([1.0, 0.0, 0.0])
        if np.allclose(a, ortho):
            ortho = np.array([0.0, 1.0, 0.0])
        v = np.cross(a, ortho)
        v = v / np.linalg.norm(v)
        # Rodrigues for 180 degrees: R = I + 2*K^2 where K is skew-symmetric of v
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + 2 * K.dot(K)

    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + K + K.dot(K) * ((1 - c) / (s ** 2))
    return rotation_matrix

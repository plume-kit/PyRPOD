import os

def ensure_dir(path):
    """
    Ensure that a directory exists. If it does not exist, create it.

    Parameters
    ----------
    path : str
        Path to the directory to ensure.
    """
    if not os.path.exists(path):
        os.makedirs(path)

def ensure_parent_dir(file_path):
    """
    Ensure that the parent directory of a file exists. If it does not exist, create it.

    Parameters
    ----------
    file_path : str
        Path to the file whose parent directory should be ensured.
    """
    parent_dir = os.path.dirname(file_path)
    if parent_dir and not os.path.exists(parent_dir):
        os.makedirs(parent_dir)
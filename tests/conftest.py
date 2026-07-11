import os
from pathlib import Path

import pytest

TESTS_DIR = Path(__file__).resolve().parent

CATEGORIES = ("unit", "integration", "verification")
GROUPS = ("mdao", "mission", "plume", "rpod")

# These files import a flat `pyrpod.<Module>` layout that no longer exists after
# the rpod -> plume/vehicle refactor. Excluded from collection until they are
# fixed or removed outright.
collect_ignore = ["old", "test_case_25.py", "rpod_verification_test_05.py"]


@pytest.fixture(autouse=True, scope="session")
def _run_from_tests_dir():
    # Legacy test cases assume the process CWD is this `tests/` directory,
    # e.g. `case_dir = '../case/...'` and `open('rpod/rpod_int_test_02...')`.
    # Reproduce that historical invocation so paths resolve without editing
    # every test file's path strings.
    previous_cwd = os.getcwd()
    os.chdir(TESTS_DIR)
    yield
    os.chdir(previous_cwd)


def pytest_collection_modifyitems(items):
    for item in items:
        stem = Path(item.fspath).stem
        relative_parts = Path(item.fspath).resolve().relative_to(TESTS_DIR).parts

        for category in CATEGORIES:
            if f"_{category}_test" in stem:
                item.add_marker(getattr(pytest.mark, category))

        for group in GROUPS:
            if group in relative_parts or stem.startswith(f"{group}_"):
                item.add_marker(getattr(pytest.mark, group))

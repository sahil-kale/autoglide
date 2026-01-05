import pytest
import sys
import os


def main():
    # Add project root to Python path so imports work
    project_root = os.path.dirname(os.path.dirname(__file__))
    sys.path.insert(0, project_root)

    # Run pytest on the whole project, discovering all tests
    ret = pytest.main([project_root])
    if ret != 0:
        print("Some tests failed.")
        sys.exit(ret)
    print("All tests passed.")


if __name__ == "__main__":
    main()

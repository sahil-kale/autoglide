import pytest
import sys
import os


def main():
    # Run pytest on the whole project, discovering all tests
    ret = pytest.main([os.path.dirname(os.path.dirname(__file__))])
    if ret != 0:
        print("Some tests failed.")
        sys.exit(ret)
    print("All tests passed.")


if __name__ == "__main__":
    main()

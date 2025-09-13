import subprocess
import os


import glob


def format_python_files(root_dir):
    exclude_dirs = {"__pycache__", "venv", ".venv", ".git", "env"}
    py_files = []
    for dirpath, dirnames, filenames in os.walk(root_dir):
        dirnames[:] = [d for d in dirnames if d not in exclude_dirs]
        for filename in filenames:
            if filename.endswith(".py"):
                py_files.append(os.path.join(dirpath, filename))
    if py_files:
        subprocess.run(["black"] + py_files)


if __name__ == "__main__":
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    format_python_files(repo_root)

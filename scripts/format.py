import subprocess
import os


def format_python_files(root_dir):
    exclude_dirs = {"__pycache__", "venv", ".venv", ".git", "env"}
    for dirpath, dirnames, filenames in os.walk(root_dir):
        # Remove excluded directories from traversal
        dirnames[:] = [d for d in dirnames if d not in exclude_dirs]
        for filename in filenames:
            if filename.endswith(".py"):
                file_path = os.path.join(dirpath, filename)
                subprocess.run(["black", file_path])


if __name__ == "__main__":
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    format_python_files(repo_root)

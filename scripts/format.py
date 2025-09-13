import subprocess
import os

def format_python_files(root_dir):
	for dirpath, dirnames, filenames in os.walk(root_dir):
		for filename in filenames:
			if filename.endswith('.py'):
				file_path = os.path.join(dirpath, filename)
				subprocess.run(['black', file_path])

if __name__ == "__main__":
	repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
	format_python_files(repo_root)
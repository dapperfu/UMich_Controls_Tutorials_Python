# Makefile for UMich Controls Tutorials Python
# Virtual environment name based on project directory
PROJECT_NAME := $(shell basename $(CURDIR))
VENV := venv_${PROJECT_NAME}
PYTHON := ${VENV}/bin/python3
PIP := ${VENV}/bin/pip
JUPYTER := ${VENV}/bin/jupyter

.PHONY: help venv install clean test run-notebook

help:
	@echo "Available targets:"
	@echo "  make venv      - Create virtual environment"
	@echo "  make install   - Install dependencies"
	@echo "  make test      - Run all notebooks to verify they work"
	@echo "  make run-notebook - Start Jupyter notebook server"
	@echo "  make clean     - Remove virtual environment and cache"

# Virtual environment target - uses actual directory as dependency
venv: ${VENV}

${VENV}:
	python3 -m venv ${VENV}
	${PIP} install --upgrade pip

# Install dependencies (editable install from pyproject.toml)
install: ${VENV}
	${PIP} install -e .

# Run all notebooks to verify they work
test: ${VENV}
	@echo "Testing all notebooks..."
	${PYTHON} -m pytest --nbval --nbval-lax --current-env || \
	${PYTHON} -c "import subprocess, sys, os; \
		notebooks = []; \
		[notebooks.extend([os.path.join(root, f) for f in files if f.endswith('.ipynb') and '.ipynb_checkpoints' not in root]) \
			for root, dirs, files in os.walk('.') if not any(d.startswith('.') for d in root.split(os.sep))]; \
		[subprocess.run([sys.executable, '-m', 'jupyter', 'nbconvert', '--to', 'notebook', '--execute', '--inplace', nb], \
			check=True, timeout=300) for nb in sorted(notebooks)]; \
		print('All notebooks executed successfully!')"

# Start Jupyter notebook server
run-notebook: ${VENV}
	${JUPYTER} notebook

# Clean up
clean:
	rm -rf ${VENV}
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true
	find . -type d -name ".ipynb_checkpoints" -exec rm -rf {} + 2>/dev/null || true


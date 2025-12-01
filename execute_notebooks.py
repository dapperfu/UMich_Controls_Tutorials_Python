#!/usr/bin/env python3
"""
Execute all Jupyter notebooks to verify they run correctly.
"""

import nbformat
from nbconvert.preprocessors import ExecutePreprocessor
from pathlib import Path
import sys


def execute_notebook(notebook_path: Path) -> tuple[bool, str]:
    """
    Execute a notebook and return success status and any error message.
    
    Args:
        notebook_path: Path to the notebook file
    
    Returns:
        Tuple of (success, error_message)
    """
    try:
        with open(notebook_path, 'r', encoding='utf-8') as f:
            nb = nbformat.read(f, as_version=4)
        
        # Use the project's venv Python if available
        import sys
        venv_python = Path(__file__).parent / 'venv_UMich_Controls_Tutorials_Python' / 'bin' / 'python3'
        if venv_python.exists():
            kernel_name = 'python3'
        else:
            kernel_name = 'python3'
        
        ep = ExecutePreprocessor(timeout=600, kernel_name=kernel_name)
        ep.preprocess(nb, {'metadata': {'path': str(notebook_path.parent)}})
        
        # Save executed notebook
        with open(notebook_path, 'w', encoding='utf-8') as f:
            nbformat.write(nb, f)
        
        return True, ""
    except Exception as e:
        return False, str(e)


def main():
    """Main function to execute all notebooks."""
    base_path = Path(__file__).parent
    notebooks = list(base_path.glob('**/*.ipynb'))
    
    # Exclude venv and other directories
    notebooks = [nb for nb in notebooks if 'venv' not in str(nb) and '.ipynb_checkpoints' not in str(nb)]
    
    total = len(notebooks)
    success_count = 0
    failed = []
    
    print(f"Executing {total} notebooks...\n")
    
    for i, notebook_path in enumerate(sorted(notebooks), 1):
        print(f"[{i}/{total}] Executing {notebook_path}...", end=' ', flush=True)
        success, error = execute_notebook(notebook_path)
        
        if success:
            print("✓")
            success_count += 1
        else:
            print(f"✗")
            print(f"  Error: {error}")
            failed.append((notebook_path, error))
    
    print(f"\n{'='*60}")
    print(f"Total: {total}")
    print(f"Success: {success_count}")
    print(f"Failed: {len(failed)}")
    
    if failed:
        print("\nFailed notebooks:")
        for path, error in failed:
            print(f"  - {path}: {error}")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())


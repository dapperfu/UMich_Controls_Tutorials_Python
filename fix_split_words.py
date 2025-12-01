#!/usr/bin/env python3
"""
Fix split words in Jupyter notebooks.

This script fixes cases where a single letter is left at the end of a line,
followed by a newline, and then the rest of the word continues on the next line.
For example: "F\n\nor instance" becomes "For instance"
"""

import json
import re
from pathlib import Path
from typing import Any


def fix_split_words(text: str) -> str:
    """
    Fix split words in text.
    
    Pattern: A single letter at the end of a line, followed by newline(s),
    then the rest of the word starting on the next line.
    
    Args:
        text: Input text to fix
        
    Returns:
        Fixed text
    """
    # Pattern: single letter at end of line, followed by newline(s), then lowercase letter
    # This matches cases like "F\n\nor" or "T\n\nhe" etc.
    # The pattern ensures:
    # 1. Single letter (A-Za-z) at end of line (before \n)
    # 2. One or more newlines (possibly with whitespace)
    # 3. Lowercase letter starting the next word
    pattern = r'([A-Za-z])\n+\s*([a-z])'
    
    def replace_func(match: re.Match[str]) -> str:
        """Join the split word."""
        first_letter = match.group(1)
        rest_of_word = match.group(2)
        return f'{first_letter}{rest_of_word}'
    
    # Apply the fix repeatedly until no more changes
    fixed_text = text
    while True:
        new_text = re.sub(pattern, replace_func, fixed_text)
        if new_text == fixed_text:
            break
        fixed_text = new_text
    
    return fixed_text


def fix_notebook(notebook_path: Path) -> bool:
    """
    Fix split words in a Jupyter notebook.
    
    Args:
        notebook_path: Path to the notebook file
        
    Returns:
        True if changes were made, False otherwise
    """
    try:
        with open(notebook_path, 'r', encoding='utf-8') as f:
            notebook = json.load(f)
    except Exception as e:
        print(f"Error reading {notebook_path}: {e}")
        return False
    
    changed = False
    
    # Process all cells
    for cell in notebook.get('cells', []):
        if cell.get('cell_type') == 'markdown':
            source = cell.get('source', [])
            if isinstance(source, list):
                # Join list of strings into single string
                text = ''.join(source)
            else:
                text = source
            
            # Fix split words
            fixed_text = fix_split_words(text)
            
            if fixed_text != text:
                # Update the cell source
                # Split back into lines for notebook format
                cell['source'] = fixed_text.splitlines(keepends=True)
                changed = True
    
    if changed:
        # Write back to file
        with open(notebook_path, 'w', encoding='utf-8') as f:
            json.dump(notebook, f, indent=1, ensure_ascii=False)
        print(f"Fixed: {notebook_path}")
        return True
    
    return False


def main() -> None:
    """Main function to fix all notebooks."""
    project_root = Path(__file__).parent
    notebook_files = list(project_root.rglob('*.ipynb'))
    
    print(f"Found {len(notebook_files)} notebook files")
    
    fixed_count = 0
    for notebook_path in notebook_files:
        if fix_notebook(notebook_path):
            fixed_count += 1
    
    print(f"\nFixed {fixed_count} notebook(s)")


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""
Script to fix image formatting in Jupyter notebooks.

Ensures all images are standalone on their own lines with blank lines above and below.
"""

import json
import re
from pathlib import Path
from typing import List


def fix_image_formatting(text: str) -> str:
    """
    Fix image formatting to ensure images are standalone with blank lines.
    
    Args:
        text: The markdown text to fix
    
    Returns:
        Fixed text with properly formatted images
    """
    # Pattern to match image markdown: ![alt](path)
    image_pattern = r'!\[([^\]]*)\]\(([^)]+)\)'
    
    lines = text.split('\n')
    result = []
    i = 0
    
    while i < len(lines):
        line = lines[i]
        
        # Check if this line contains an image
        if re.search(image_pattern, line):
            # Check if there's text before the image on the same line
            image_match = re.search(image_pattern, line)
            if image_match:
                before_image = line[:image_match.start()].strip()
                after_image = line[image_match.end():].strip()
                image_text = image_match.group(0)
                
                # If there's text before the image, add it as a separate line
                if before_image:
                    result.append(before_image)
                    # Ensure blank line before image
                    if result and result[-1] != '':
                        result.append('')
                
                # Add the image on its own line
                result.append(image_text)
                
                # If there's text after the image, handle it
                if after_image:
                    # Ensure blank line after image
                    result.append('')
                    result.append(after_image)
                else:
                    # Ensure blank line after image if next line is not blank
                    if i + 1 < len(lines) and lines[i + 1].strip() != '':
                        result.append('')
            else:
                # Just an image line, ensure blank lines around it
                if result and result[-1].strip() != '':
                    result.append('')
                result.append(line)
                if i + 1 < len(lines) and lines[i + 1].strip() != '':
                    result.append('')
        else:
            result.append(line)
        
        i += 1
    
    # Join and clean up multiple consecutive blank lines (max 2)
    text_result = '\n'.join(result)
    # Replace 3+ consecutive newlines with 2
    text_result = re.sub(r'\n{3,}', '\n\n', text_result)
    
    return text_result


def fix_notebook_images(notebook_path: Path) -> tuple[bool, int]:
    """
    Fix image formatting in a notebook file.
    
    Args:
        notebook_path: Path to the notebook file
    
    Returns:
        Tuple of (changed, num_fixes) where changed is True if file was modified
    """
    with open(notebook_path, 'r', encoding='utf-8') as f:
        notebook = json.load(f)
    
    changed = False
    num_fixes = 0
    
    for cell in notebook['cells']:
        if cell['cell_type'] != 'markdown':
            continue
        
        source = ''.join(cell['source'])
        fixed_source = fix_image_formatting(source)
        
        if fixed_source != source:
            changed = True
            num_fixes += 1
            # Convert back to list of strings (Jupyter format)
            cell['source'] = [line + '\n' for line in fixed_source.split('\n')]
            # Fix last line (shouldn't have trailing newline if empty)
            if cell['source'] and cell['source'][-1] == '\n':
                # Check if it's just a newline
                if len(cell['source']) > 1 or cell['source'][-1] != '\n':
                    # Remove trailing newline from last line if it's empty
                    if cell['source'][-1] == '\n':
                        cell['source'][-1] = ''
                    else:
                        # Remove trailing newline but keep content
                        cell['source'][-1] = cell['source'][-1].rstrip('\n')
    
    if changed:
        with open(notebook_path, 'w', encoding='utf-8') as f:
            json.dump(notebook, f, indent=1, ensure_ascii=False)
    
    return changed, num_fixes


def main():
    """Main function to process all notebooks."""
    base_path = Path(__file__).parent
    notebooks = list(base_path.glob('**/*.ipynb'))
    
    # Exclude venv and other directories
    notebooks = [nb for nb in notebooks if 'venv' not in str(nb) and '.ipynb_checkpoints' not in str(nb)]
    
    total_fixes = 0
    files_changed = 0
    
    for notebook_path in sorted(notebooks):
        changed, num_fixes = fix_notebook_images(notebook_path)
        if changed:
            files_changed += 1
            total_fixes += num_fixes
            print(f"Fixed {num_fixes} cell(s) in {notebook_path}")
    
    print(f"\nTotal: {files_changed} files changed, {total_fixes} cells fixed")


if __name__ == '__main__':
    main()


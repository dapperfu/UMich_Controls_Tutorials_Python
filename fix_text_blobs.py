#!/usr/bin/env python3
"""
Script to fix text blobs in Jupyter notebooks by breaking up long paragraphs.

Looks for paragraphs longer than 500 characters and breaks them at sentence
boundaries, ensuring proper double newlines for Jupyter markdown rendering.
"""

import json
import re
from pathlib import Path
from typing import List, Tuple


def break_long_paragraph(text: str, max_length: int = 500) -> str:
    """
    Break a long paragraph into multiple paragraphs at sentence boundaries.
    
    Args:
        text: The text to break up
        max_length: Maximum length before breaking (default 500)
    
    Returns:
        Text with paragraphs broken up
    """
    if len(text) <= max_length:
        return text
    
    # Split into sentences more carefully
    # Look for sentence endings followed by space and capital letter
    # Pattern: end punctuation, optional quote, whitespace, capital letter
    sentence_pattern = r'([.!?])(["\']?)\s+([A-Z])'
    
    # Find all sentence boundaries
    matches = list(re.finditer(sentence_pattern, text))
    
    if not matches:
        # No clear sentence boundaries, try to split on other punctuation
        # This is a fallback
        parts = re.split(r'([,;:])\s+', text)
        if len(parts) > 1:
            result = []
            current = ''
            for i in range(0, len(parts), 2):
                part = parts[i] + (parts[i+1] if i+1 < len(parts) else '')
                if len(current + part) > max_length and current:
                    result.append(current.strip())
                    current = part
                else:
                    current += part
            if current:
                result.append(current.strip())
            return '\n\n'.join(result)
        return text
    
    # Build sentences from matches
    result = []
    last_end = 0
    
    for match in matches:
        sentence_end = match.end() - len(match.group(3))  # End before the capital letter
        sentence = text[last_end:sentence_end].strip()
        
        if sentence:
            # Check if adding this sentence would exceed max_length
            if result and len(result[-1] + ' ' + sentence) <= max_length:
                # Combine with previous paragraph
                result[-1] += ' ' + sentence
            else:
                # Start new paragraph
                result.append(sentence)
        
        last_end = sentence_end
    
    # Add remaining text
    if last_end < len(text):
        remaining = text[last_end:].strip()
        if remaining:
            if result and len(result[-1] + ' ' + remaining) <= max_length:
                result[-1] += ' ' + remaining
            else:
                result.append(remaining)
    
    return '\n\n'.join(result)


def fix_notebook_blobs(notebook_path: Path, max_length: int = 500) -> Tuple[bool, int]:
    """
    Fix text blobs in a notebook file.
    
    Args:
        notebook_path: Path to the notebook file
        max_length: Maximum paragraph length before breaking
    
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
        
        # Split by double newlines to get paragraphs
        paragraphs = source.split('\n\n')
        new_paragraphs = []
        
        for para in paragraphs:
            # Remove single newlines within paragraphs (they become spaces)
            para_clean = para.replace('\n', ' ').strip()
            
            # Skip empty paragraphs
            if not para_clean:
                new_paragraphs.append('')
                continue
            
            # Check if paragraph is too long
            if len(para_clean) > max_length:
                # Break it up
                broken = break_long_paragraph(para_clean, max_length)
                new_paragraphs.append(broken)
                changed = True
                num_fixes += 1
            else:
                new_paragraphs.append(para_clean)
        
        if changed:
            # Reconstruct source with proper formatting
            new_source = '\n\n'.join(new_paragraphs)
            # Convert back to list of strings (Jupyter format)
            cell['source'] = [line + '\n' for line in new_source.split('\n')]
            # Fix last line (shouldn't have trailing newline)
            if cell['source'] and cell['source'][-1] == '\n':
                cell['source'][-1] = cell['source'][-1].rstrip('\n')
            if not cell['source'][-1]:
                cell['source'] = cell['source'][:-1]
    
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
        changed, num_fixes = fix_notebook_blobs(notebook_path)
        if changed:
            files_changed += 1
            total_fixes += num_fixes
            print(f"Fixed {num_fixes} blob(s) in {notebook_path}")
    
    print(f"\nTotal: {files_changed} files changed, {total_fixes} blobs fixed")


if __name__ == '__main__':
    main()


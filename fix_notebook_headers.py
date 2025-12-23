#!/usr/bin/env python3
"""Fix header formatting in Jupyter notebooks.

This script identifies markdown cells that should be headers (# for main titles,
## for section headers) and converts them to proper markdown header format.
"""

import json
import re
from pathlib import Path
from typing import Any, Dict, List, Tuple


def is_main_title(text: str, cell_index: int, previous_cell_type: str) -> bool:
    """Check if a cell should be a main title (# header).
    
    Parameters
    ----------
    text : str
        The cell text content
    cell_index : int
        Index of the cell in the notebook
    previous_cell_type : str
        Type of the previous cell ('code' or 'markdown')
        
    Returns
    -------
    bool
        True if this should be a main title
    """
    # Main title is typically the first markdown cell after imports
    if cell_index > 0 and previous_cell_type != 'code':
        return False
    
    # Check for title patterns
    text_clean = text.strip()
    
    # Pattern: "System Name: Section Name" (e.g., "Cruise Control: System Modeling")
    if re.match(r'^[A-Z][^:]+:\s+(System\s+(Modeling|Analysis)|(PID|Root\s+Locus|Frequency|State\s+Space|Digital)\s+Controller?\s+Design)', text_clean, re.IGNORECASE):
        return True
    
    # Pattern: "DC Motor Speed: System Modeling" or similar
    if re.match(r'^[A-Z][^:]+:\s+(System\s+(Modeling|Analysis)|Control)', text_clean, re.IGNORECASE):
        return True
    
    # Pattern: "Introduction: System Modeling"
    if re.match(r'^Introduction:\s+System\s+(Modeling|Analysis)', text_clean, re.IGNORECASE):
        return True
    
    return False


def is_section_header(text: str) -> bool:
    """Check if a cell should be a section header (## header).
    
    Parameters
    ----------
    text : str
        The cell text content
        
    Returns
    -------
    bool
        True if this should be a section header
    """
    text_clean = text.strip()
    
    # Common section header patterns
    section_patterns = [
        r'^Physical\s+setup$',
        r'^System\s+equations?$',
        r'^System\s+parameters?$',
        r'^System\s+model\s+and\s+parameters$',
        r'^State-space\s+model$',
        r'^Transfer\s+function\s+model$',
        r'^State\s+space\s+model$',
        r'^Modelica\s+Model$',
        r'^Key\s+MATLAB\s+commands',
        r'^Design\s+requirements?$',
        r'^PID\s+controller\s+design$',
        r'^Root\s+locus\s+design$',
        r'^Frequency\s+response\s+design$',
        r'^State\s+feedback\s+design$',
        r'^Digital\s+controller\s+design$',
        r'^Open-loop\s+response$',
        r'^Closed-loop\s+response$',
        r'^Controller\s+design$',
        r'^System\s+identification$',
    ]
    
    for pattern in section_patterns:
        if re.match(pattern, text_clean, re.IGNORECASE):
            return True
    
    # Check if it's a short standalone text (likely a header)
    # But exclude cells that are clearly not headers (like parameter lists)
    if len(text_clean) < 50 and not text_clean.startswith('(') and not text_clean.startswith('['):
        # Check if it looks like a title (capitalized, no punctuation at end, or ends with colon)
        if (text_clean[0].isupper() and 
            (not text_clean.endswith('.') or text_clean.endswith(':')) and
            '\n' not in text_clean):
            return True
    
    return False


def extract_embedded_header(text: str) -> Tuple[str, str]:
    """Extract section header from text that may have header embedded in it.
    
    Parameters
    ----------
    text : str
        The cell text content
        
    Returns
    -------
    Tuple[str, str]
        (header_text, remaining_text) or (None, text) if no header found
    """
    text_clean = text.strip()
    
    # Check for headers at the start of the text
    section_patterns = [
        (r'^(Physical\s+setup)\s*\n\n', 'Physical setup'),
        (r'^(System\s+equations?)\s*\n\n', 'System equations'),
        (r'^(System\s+parameters?)\s*\n\n', 'System parameters'),
        (r'^(State-space\s+model)\s*\n\n', 'State-space model'),
        (r'^(Transfer\s+function\s+model)\s*\n\n', 'Transfer function model'),
    ]
    
    for pattern, header_name in section_patterns:
        match = re.match(pattern, text_clean, re.IGNORECASE | re.MULTILINE)
        if match:
            remaining = text_clean[match.end():].strip()
            return header_name, remaining
    
    # Check for header at the end of a paragraph (like "System equations" at end)
    # Pattern: text ending with "System equations" on its own line
    end_header_patterns = [
        (r'\n\n(System\s+equations?)\s*$', 'System equations'),
        (r'\n\n(Physical\s+setup)\s*$', 'Physical setup'),
    ]
    
    for pattern, header_name in end_header_patterns:
        match = re.search(pattern, text_clean, re.IGNORECASE | re.MULTILINE)
        if match:
            remaining = text_clean[:match.start()].strip()
            return header_name, remaining
    
    return None, text


def fix_notebook_headers(notebook_path: Path) -> bool:
    """Fix header formatting in a notebook.
    
    Parameters
    ----------
    notebook_path : Path
        Path to the notebook file
        
    Returns
    -------
    bool
        True if changes were made, False otherwise
    """
    with open(notebook_path, 'r', encoding='utf-8') as f:
        notebook = json.load(f)
    
    notebook_changed = False
    previous_cell_type = None
    
    for i, cell in enumerate(notebook['cells']):
        if cell.get('cell_type') != 'markdown':
            previous_cell_type = cell.get('cell_type')
            continue
        
        source_lines = cell.get('source', [])
        if not source_lines:
            previous_cell_type = 'markdown'
            continue
        
        # Join source lines to get full text
        full_text = ''.join(source_lines)
        text_clean = full_text.strip()
        
        # Skip empty cells
        if not text_clean:
            previous_cell_type = 'markdown'
            continue
        
        # Check if this should be a main title
        if is_main_title(text_clean, i, previous_cell_type):
            # Convert to # header
            if not text_clean.startswith('#'):
                cell['source'] = [f'# {text_clean}\n']
                notebook_changed = True
                previous_cell_type = 'markdown'
                continue
        
        # Check if this should be a section header
        if is_section_header(text_clean):
            # Convert to ## header
            if not text_clean.startswith('#'):
                cell['source'] = [f'## {text_clean}\n']
                notebook_changed = True
                previous_cell_type = 'markdown'
                continue
        
        # Check for embedded headers (header + content in same cell)
        header_text, remaining_text = extract_embedded_header(text_clean)
        if header_text:
            # Split into two cells: header cell and content cell
            # Create new header cell
            header_cell = {
                'cell_type': 'markdown',
                'metadata': cell.get('metadata', {}),
                'source': [f'## {header_text}\n']
            }
            
            # Update current cell with remaining content
            if remaining_text:
                cell['source'] = [remaining_text + '\n']
            else:
                # If no remaining text, remove the cell (will be replaced by header)
                cell['source'] = [f'## {header_text}\n']
            
            # Insert header cell before current cell
            notebook['cells'].insert(i, header_cell)
            notebook_changed = True
            previous_cell_type = 'markdown'
            continue
        
        previous_cell_type = 'markdown'
    
    if notebook_changed:
        with open(notebook_path, 'w', encoding='utf-8') as f:
            json.dump(notebook, f, indent=1, ensure_ascii=False)
    
    return notebook_changed


def main() -> None:
    """Fix headers in all notebooks."""
    project_root = Path(__file__).parent
    
    # Find all notebooks
    notebooks = list(project_root.glob('**/*.ipynb'))
    # Exclude notebooks in hidden directories
    notebooks = [nb for nb in notebooks if '__pycache__' not in str(nb) and 
                 not any(p.startswith('.') for p in nb.parts if p != '.')]
    
    fixed_count = 0
    unchanged_count = 0
    
    for notebook_path in sorted(notebooks):
        print(f"Processing {notebook_path}...")
        if fix_notebook_headers(notebook_path):
            print(f"  ✓ Fixed {notebook_path}")
            fixed_count += 1
        else:
            print(f"  - No changes needed for {notebook_path}")
            unchanged_count += 1
    
    print(f"\nSummary: {fixed_count} notebooks fixed, {unchanged_count} unchanged")


if __name__ == '__main__':
    main()


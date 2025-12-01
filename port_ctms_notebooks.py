#!/usr/bin/env python3
"""
Script to port CTMS MATLAB tutorials to Python Jupyter notebooks.

Extracts MATLAB code from CTMS HTML files (between ##### SOURCE BEGIN ##### and ##### SOURCE END #####)
and converts them to Python notebooks following existing patterns.
"""

import re
import json
from pathlib import Path
from typing import List, Tuple, Dict
import html


def extract_matlab_source(html_file: Path) -> str:
    """Extract MATLAB source code from CTMS HTML file."""
    content = html_file.read_text(encoding='utf-8', errors='ignore')
    
    # Find content between SOURCE BEGIN and SOURCE END
    begin_match = re.search(r'##### SOURCE BEGIN #####', content)
    end_match = re.search(r'##### SOURCE END #####', content)
    
    if not begin_match or not end_match:
        return ""
    
    source = content[begin_match.end():end_match.start()]
    return source.strip()


def parse_matlab_source(source: str) -> List[Tuple[str, str]]:
    """
    Parse MATLAB source into cells.
    Returns list of (cell_type, content) tuples where cell_type is 'markdown' or 'code'.
    """
    cells = []
    lines = source.split('\n')
    current_cell = []
    current_type = None
    
    for line in lines:
        # MATLAB comment line (starts with %)
        if line.strip().startswith('%'):
            # If we were in a code cell, finalize it
            if current_type == 'code' and current_cell:
                cells.append(('code', '\n'.join(current_cell)))
                current_cell = []
            
            # Remove leading % and whitespace
            comment = line.lstrip('%').lstrip()
            
            # Handle special markers
            if comment.strip() == '':
                # Empty comment line - section break
                if current_cell:
                    cells.append(('markdown', '\n'.join(current_cell)))
                    current_cell = []
            elif comment.strip().startswith('%%'):
                # Section header (%% Title)
                if current_cell:
                    cells.append(('markdown', '\n'.join(current_cell)))
                    current_cell = []
                # Convert %% Title to ## Title (use ## for subsections)
                title = comment.lstrip('%').strip()
                # Check if it's a main title (first %% in file or after empty line)
                if not cells or cells[-1][0] == 'code':
                    current_cell.append(f"# {title}")
                else:
                    current_cell.append(f"## {title}")
            elif comment.strip().startswith('<<'):
                # Figure reference: <<Content/Path/to/figure.png>>
                # Convert to markdown image syntax
                fig_match = re.search(r'<<([^>]+)>>', comment)
                if fig_match:
                    fig_path = fig_match.group(1)
                    # Extract just the filename
                    fig_name = Path(fig_path).name
                    current_cell.append(f"![{fig_name}](figures/{fig_name})")
            else:
                # Regular comment - add to markdown
                current_cell.append(comment)
            
            current_type = 'markdown'
        else:
            # Code line
            if current_type == 'markdown' and current_cell:
                cells.append(('markdown', '\n'.join(current_cell)))
                current_cell = []
            
            current_cell.append(line)
            current_type = 'code'
    
    # Finalize last cell
    if current_cell:
        cells.append((current_type, '\n'.join(current_cell)))
    
    return cells


def convert_matlab_to_python(code: str) -> str:
    """Convert MATLAB code to Python code."""
    lines = code.split('\n')
    python_lines = []
    
    for line in lines:
        # Skip empty lines
        if not line.strip():
            python_lines.append('')
            continue
        
        # Convert common MATLAB patterns
        python_line = line
        
        # Remove trailing semicolons
        python_line = python_line.rstrip(';')
        
        # Convert exponentiation: ^ -> **
        python_line = python_line.replace('^', '**')
        
        # Convert MATLAB array indexing FIRST (before range conversion)
        # arr(1:2) -> arr[0:2] (1-based to 0-based, end inclusive to exclusive)
        def convert_matlab_indexing(match):
            var = match.group(1)
            indices = match.group(2)
            if ':' in indices:
                parts = indices.split(':')
                if len(parts) == 2:
                    start = parts[0].strip()
                    end = parts[1].strip()
                    try:
                        start_val = int(start) - 1 if start else 0
                        end_val = int(end) + 1 if end else None  # +1 because end is exclusive
                        return f'{var}[{start_val}:{end_val}]'
                    except ValueError:
                        # If not pure numbers, use expressions
                        start_py = f'({start} - 1)' if start else '0'
                        end_py = f'({end} + 1)' if end else ''
                        return f'{var}[{start_py}:{end_py}]' if end_py else f'{var}[{start_py}:]'
            else:
                # Single index: arr(1) -> arr[0]
                try:
                    idx = int(indices) - 1
                    return f'{var}[{idx}]'
                except ValueError:
                    return f'{var}[{indices} - 1]'
        
        # Convert MATLAB indexing: variable(1:2) -> variable[0:2]
        # Only match simple numeric indices to avoid matching function calls
        # Use word boundary to ensure we match variable names properly
        # Protect the result with a placeholder to avoid further conversions
        indexing_placeholders = {}
        placeholder_idx = [0]
        
        def convert_and_protect(match):
            result = convert_matlab_indexing(match)
            placeholder = f"__IDX_{placeholder_idx[0]}__"
            placeholder_idx[0] += 1
            indexing_placeholders[placeholder] = result
            return placeholder
        
        python_line = re.sub(r'\b(\w+)\(([0-9:]+)\)', convert_and_protect, python_line)
        
        # Convert MATLAB range syntax (standalone, not in brackets): start:step:end -> np.arange(start, end, step)
        # Also handle start:end -> np.arange(start, end+1, 1)
        def convert_range_standalone(match):
            range_str = match.group(0)
            if ':' in range_str:
                parts = [p.strip() for p in range_str.split(':')]
                if len(parts) == 2:
                    # start:end -> np.arange(start, end+1, 1) but usually means np.arange(start, end+1)
                    return f'np.arange({parts[0]}, {parts[1]}+1)'
                elif len(parts) == 3:
                    # start:step:end -> np.arange(start, end, step)
                    return f'np.arange({parts[0]}, {parts[2]}, {parts[1]})'
            return match.group(0)
        
        # Convert standalone range syntax (not inside brackets or function calls)
        # Match patterns like "0:0.1:20" or "0:10" that are standalone
        python_line = re.sub(r'\b([0-9.]+:[0-9.]+:[0-9.]+)\b', convert_range_standalone, python_line)
        python_line = re.sub(r'\b([0-9.]+:[0-9.]+)\b(?!\s*[,\]])', convert_range_standalone, python_line)
        
        # Convert MATLAB array range syntax [start:step:end] -> np.arange(start, end, step)
        # BUT: if this is already inside np.array([...]), don't double-wrap
        # We'll handle ranges BEFORE array conversion to avoid double wrapping
        def convert_range_syntax(match):
            range_str = match.group(1)
            if ':' in range_str:
                parts = [p.strip() for p in range_str.split(':')]
                if len(parts) == 2:
                    return f'np.arange({parts[0]}, {parts[1]}+1)'
                elif len(parts) == 3:
                    return f'np.arange({parts[0]}, {parts[2]}, {parts[1]})'
            return match.group(0)
        
        # Match [number:number:number] or [number:number] inside array brackets
        # But skip if already inside np.array
        if 'np.array' not in python_line:
            python_line = re.sub(r'\[([0-9.]+:[0-9.]+:[0-9.]+)\]', convert_range_syntax, python_line)
            python_line = re.sub(r'\[([0-9.]+:[0-9.]+)\]', convert_range_syntax, python_line)
        
        # tf('s') -> control.tf('s')
        python_line = re.sub(r'\btf\s*\(', 'control.tf(', python_line)
        
        # ss(A,B,C,D) -> control.StateSpace(A, B, C, D)
        python_line = re.sub(r'\bss\s*\(', 'control.StateSpace(', python_line)
        
        # Convert MATLAB array syntax
        # First handle multi-row arrays [a; b; c] -> np.array([[a], [b], [c]])
        def convert_matlab_array_inline(match):
            arr_content = match.group(1)
            # If content is already np.arange(...), extract it without brackets
            if 'np.arange' in arr_content:
                # Extract just the np.arange expression
                arange_match = re.search(r'np\.arange\([^)]+\)', arr_content)
                if arange_match:
                    return arange_match.group(0)  # Return just np.arange(...) without brackets
            
            # Skip if it's a pure numeric range (will be handled by range conversion)
            if ':' in arr_content and not any(c.isalpha() for c in arr_content.replace(':', '').replace('.', '').replace('-', '').replace(' ', '')):
                # This is a range - convert it first
                parts = [p.strip() for p in arr_content.split(':')]
                if len(parts) == 3:
                    range_expr = f'np.arange({parts[0]}, {parts[2]}, {parts[1]})'
                    return range_expr
                elif len(parts) == 2:
                    range_expr = f'np.arange({parts[0]}, {parts[1]}+1)'
                    return range_expr
            
            if ';' in arr_content:
                # Multi-row array
                rows = [r.strip() for r in arr_content.split(';') if r.strip()]
                row_arrays = []
                for row in rows:
                    elements = [e.strip() for e in row.split() if e.strip()]
                    if elements:
                        row_arrays.append('[' + ', '.join(elements) + ']')
                if row_arrays:
                    return 'np.array([' + ', '.join(row_arrays) + '])'
            else:
                # Single row array [a b c] -> np.array([a, b, c])
                elements = [e.strip() for e in arr_content.split() if e.strip()]
                if elements:
                    return 'np.array([' + ', '.join(elements) + '])'
            return match.group(0)  # Return unchanged if can't convert
        
        # Convert [a b c] or [a; b; c] patterns (but not ranges that are already np.arange)
        python_line = re.sub(r'\[([^\]]+)\]', convert_matlab_array_inline, python_line)
        
        # Fix invalid assignments like "np.array([r,p,k]) = ..." -> "r, p, k = ..."
        python_line = re.sub(r'np\.array\(\[([^\]]+)\]\)\s*=', lambda m: ', '.join(e.strip() for e in m.group(1).split(',')) + ' =', python_line)
        
        # Fix missing closing parentheses in array definitions
        # This is a heuristic - if we see np.array([... without closing, try to fix
        # But this is risky, so we'll be conservative
        
        # Handle multi-line array definitions that might be missing closing brackets
        # This is complex and might need manual fixing, but we can try
        
        # step(sys) -> T, yout = control.step_response(sys)
        python_line = re.sub(r'\bstep\s*\(([^)]+)\)', r'T, yout = control.step_response(\1)', python_line)
        
        # bode(sys) -> control.bode_plot(sys)
        python_line = re.sub(r'\bbode\s*\(', 'control.bode_plot(', python_line)
        
        # rlocus(sys) -> control.root_locus(sys)
        python_line = re.sub(r'\brlocus\s*\(', 'control.root_locus(', python_line)
        
        # feedback(sys1, sys2) -> control.feedback(sys1, sys2)
        python_line = re.sub(r'\bfeedback\s*\(', 'control.feedback(', python_line)
        
        # pole(sys) -> sys.poles() (use method on system object, control.pole may not exist)
        def convert_pole(match):
            arg = match.group(1).strip()
            return f'{arg}.poles()'
        python_line = re.sub(r'\bpole\s*\(([^)]+)\)', convert_pole, python_line)
        # Also handle control.pole() if it appears
        python_line = re.sub(r'control\.pole\s*\(([^)]+)\)', lambda m: f'{m.group(1).strip()}.poles()', python_line)
        
        # zero(sys) -> sys.zeros() (method on system object)
        def convert_zero(match):
            arg = match.group(1).strip()
            return f'{arg}.zeros()'
        python_line = re.sub(r'\bzero\s*\(([^)]+)\)', convert_zero, python_line)
        
        # zpk(sys) -> display zeros, poles, gain (MATLAB zpk displays, doesn't create)
        # For Python, we'll create a zpk system from the extracted values
        # But first check if it's already control.zpk to avoid double conversion
        if 'control.zpk' not in python_line:
            def convert_zpk_sys(match):
                sys_arg = match.group(1).strip()
                # Extract zeros, poles, and gain from system and create zpk system
                # Use intermediate variable to avoid issues
                return f'control.zpk(({sys_arg}).zeros(), ({sys_arg}).poles(), float(({sys_arg}).dcgain()))'
            
            # Handle zpk(sys) - single argument (system), but not if already control.zpk
            python_line = re.sub(r'(?<!control\.)\bzpk\s*\(([^,)]+)\)', convert_zpk_sys, python_line)
            
            # zpk(z, p, k) -> control.zpk(z, p, k) (zero-pole-gain form with explicit args)
            python_line = re.sub(r'(?<!control\.)\bzpk\s*\(', 'control.zpk(', python_line)
        
        # ctrb(A, B) -> control.ctrb(A, B) (controllability matrix)
        python_line = re.sub(r'\bctrb\s*\(', 'control.ctrb(', python_line)
        
        # obsv(A, C) -> control.obsv(A, C) (observability matrix)
        python_line = re.sub(r'\bobsv\s*\(', 'control.obsv(', python_line)
        
        # lqr(A, B, Q, R) -> control.lqr(A, B, Q, R) (linear quadratic regulator)
        python_line = re.sub(r'\blqr\s*\(', 'control.lqr(', python_line)
        
        # rank(matrix) -> np.linalg.matrix_rank(matrix)
        python_line = re.sub(r'\brank\s*\(', 'np.linalg.matrix_rank(', python_line)
        
        # c2d(sys, Ts, method) -> control.c2d(sys, Ts, method) (continuous to discrete)
        python_line = re.sub(r'\bc2d\s*\(', 'control.c2d(', python_line)
        
        # zgrid(zeta, wn) -> control.zgrid(zeta, wn) or comment out if not available
        if 'zgrid' in python_line and 'control.zgrid' not in python_line:
            python_line = python_line.replace('zgrid', '# zgrid')  # Comment out for now
        
        # d2c(sys, method) -> control.d2c(sys, method) (discrete to continuous)
        python_line = re.sub(r'\bd2c\s*\(', 'control.d2c(', python_line)
        
        # MATLAB transpose: A' -> A.T (but be careful with strings)
        # We need to avoid converting ' inside string literals
        # Strategy: protect string literals first, then convert transpose, then restore strings
        string_placeholders = {}
        placeholder_counter = [0]
        
        def protect_string(match):
            placeholder = f"__STRING_{placeholder_counter[0]}__"
            placeholder_counter[0] += 1
            string_placeholders[placeholder] = match.group(0)
            return placeholder
        
        # Protect string literals (both single and double quoted)
        protected_line = re.sub(r'(["\'])(?:(?=(\\?))\2.)*?\1', protect_string, python_line)
        
        # Now convert transpose on the protected line
        # Convert identifier' -> identifier.T
        protected_line = re.sub(r'([a-zA-Z_][a-zA-Z0-9_\[\]()]*)\'', r'\1.T', protected_line)
        # Convert )' -> ).T
        protected_line = re.sub(r'\)\'', r').T', protected_line)
        # Convert ]' -> ].T
        protected_line = re.sub(r'\]\'', r'].T', protected_line)
        
        # Restore string literals
        for placeholder, original in string_placeholders.items():
            protected_line = protected_line.replace(placeholder, original)
        
        python_line = protected_line
        
        
        # residue() -> scipy.signal.residue() (partial fraction decomposition)
        python_line = re.sub(r'\bresidue\s*\(', 'scipy.signal.residue(', python_line)
        
        # Add scipy import if residue is used
        if 'scipy.signal.residue' in python_line:
            # Will be handled in imports
            pass
        
        # sgrid(zeta, wn) -> control.sgrid(zeta, wn) (if control library supports it)
        # For now, comment it out as it may not be available
        if 'sgrid' in python_line and 'control.sgrid' not in python_line:
            python_line = python_line.replace('sgrid', '# sgrid')  # Comment out for now
        
        # axis([x1, x2, y1, y2]) -> plt.axis([x1, x2, y1, y2])
        python_line = re.sub(r'\baxis\s*\(', 'plt.axis(', python_line)
        
        # grid -> plt.grid()
        if re.match(r'^\s*grid\s*$', python_line):
            python_line = 'plt.grid()'
        
        # title('text') -> plt.title('text')
        python_line = re.sub(r'\btitle\s*\(', 'plt.title(', python_line)
        
        # xlabel('text') -> plt.xlabel('text')
        python_line = re.sub(r'\bxlabel\s*\(', 'plt.xlabel(', python_line)
        
        # ylabel('text') -> plt.ylabel('text')
        python_line = re.sub(r'\bylabel\s*\(', 'plt.ylabel(', python_line)
        
        # legend -> plt.legend()
        if re.match(r'^\s*legend\s*$', python_line):
            python_line = 'plt.legend()'
        
        # Restore indexing placeholders (after all other conversions)
        for placeholder, original in indexing_placeholders.items():
            python_line = python_line.replace(placeholder, original)
        
        python_lines.append(python_line)
    
    return '\n'.join(python_lines)


def convert_matlab_array(array_str: str) -> str:
    """Convert MATLAB array syntax to NumPy array."""
    # Handle semicolon-separated rows
    if ';' in array_str:
        rows = array_str.split(';')
        row_strs = []
        for row in rows:
            row = row.strip()
            if row:
                # Convert space-separated to list
                elements = row.split()
                row_strs.append('[' + ', '.join(elements) + ']')
        return 'np.array([' + ', '.join(row_strs) + '])'
    else:
        # Single row
        elements = array_str.split()
        return 'np.array([' + ', '.join(elements) + '])'


def create_notebook(cells: List[Tuple[str, str]], title: str) -> dict:
    """Create a Jupyter notebook structure from parsed cells."""
    notebook_cells = []
    
    # Add imports cell if code cells exist
    has_code = any(cell_type == 'code' for cell_type, _ in cells)
    if has_code:
        imports = """import control
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import scipy.signal

sns.set_style("whitegrid")"""
        # Jupyter stores source as list of strings, each line is a separate string
        notebook_cells.append({
            "cell_type": "code",
            "execution_count": None,
            "metadata": {},
            "outputs": [],
            "source": imports.splitlines(keepends=True)
        })
    
    execution_count = 1
    for cell_type, content in cells:
        if cell_type == 'markdown':
                # Jupyter stores source as list of strings, each line is a separate string
                notebook_cells.append({
                    "cell_type": "markdown",
                    "metadata": {
                        "slideshow": {
                            "slide_type": "notes"
                        }
                    },
                    "source": content.splitlines(keepends=True) if content else ['']
                })
        elif cell_type == 'code':
            # Convert MATLAB to Python
            python_code = convert_matlab_to_python(content)
            if python_code.strip():  # Only add non-empty code cells
                # Add plt.show() after plotting commands if not present
                lines = python_code.split('\n')
                has_plot = any('plot' in line or 'bode' in line or 'root_locus' in line or 'step_response' in line for line in lines)
                if has_plot and 'plt.show()' not in python_code:
                    lines.append('plt.show()')
                    python_code = '\n'.join(lines)
                
                # Jupyter stores source as list of strings, each line is a separate string
                # splitlines(keepends=True) preserves newlines
                notebook_cells.append({
                    "cell_type": "code",
                    "execution_count": execution_count,
                    "metadata": {
                        "slideshow": {
                            "slide_type": "-"
                        }
                    },
                    "outputs": [],
                    "source": python_code.splitlines(keepends=True) if python_code else ['']
                })
                execution_count += 1
    
    return {
        "cells": notebook_cells,
        "metadata": {
            "kernelspec": {
                "display_name": "Python 3",
                "language": "python",
                "name": "python3"
            },
            "language_info": {
                "name": "python",
                "version": "3.12.0"
            },
            "nbconvert_exporter": "python"
        },
        "nbformat": 4,
        "nbformat_minor": 4
    }


def port_notebook(example: str, section: str, ctms_dir: Path, output_dir: Path) -> bool:
    """Port a single CTMS tutorial to a Python notebook."""
    # Find CTMS HTML file
    html_file = ctms_dir / f"index.php?example={example}&section={section}"
    
    if not html_file.exists():
        print(f"  ERROR: CTMS file not found: {html_file}")
        return False
    
    # Extract MATLAB source
    source = extract_matlab_source(html_file)
    if not source:
        print(f"  WARNING: No source found in {html_file}")
        return False
    
    # Parse into cells
    cells = parse_matlab_source(source)
    if not cells:
        print(f"  WARNING: No cells parsed from {html_file}")
        return False
    
    # Create notebook
    title = f"{example}: {section}"
    notebook = create_notebook(cells, title)
    
    # Ensure output directory exists
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Write notebook
    notebook_file = output_dir / f"{example}_{section}.ipynb"
    with open(notebook_file, 'w', encoding='utf-8') as f:
        json.dump(notebook, f, indent=1, ensure_ascii=False)
    
    print(f"  Created: {notebook_file}")
    return True


def main():
    """Main function to port all missing notebooks."""
    base_dir = Path(__file__).parent
    ctms_dir = base_dir / "ctms" / "ctms.engin.umich.edu" / "CTMS"
    
    # Missing notebooks to port
    missing = [
        # AircraftPitch
        ("AircraftPitch", "SystemModeling"),
        ("AircraftPitch", "SystemAnalysis"),
        ("AircraftPitch", "ControlPID"),
        ("AircraftPitch", "ControlRootLocus"),
        ("AircraftPitch", "ControlStateSpace"),
        ("AircraftPitch", "ControlDigital"),
        # CruiseControl
        ("CruiseControl", "ControlDigital"),
        ("CruiseControl", "ControlRootLocus"),
        # MotorSpeed
        ("MotorSpeed", "ControlDigital"),
        ("MotorSpeed", "ControlFrequency"),
        ("MotorSpeed", "ControlPID"),
        ("MotorSpeed", "ControlRootLocus"),
        ("MotorSpeed", "ControlStateSpace"),
        ("MotorSpeed", "SystemAnalysis"),
        # MotorPosition
        ("MotorPosition", "SystemModeling"),
        ("MotorPosition", "SystemAnalysis"),
        ("MotorPosition", "ControlPID"),
        ("MotorPosition", "ControlRootLocus"),
        ("MotorPosition", "ControlFrequency"),
        ("MotorPosition", "ControlStateSpace"),
        ("MotorPosition", "ControlDigital"),
        # Suspension
        ("Suspension", "SystemModeling"),
        ("Suspension", "SystemAnalysis"),
        ("Suspension", "ControlPID"),
        ("Suspension", "ControlRootLocus"),
        ("Suspension", "ControlFrequency"),
        ("Suspension", "ControlStateSpace"),
        ("Suspension", "ControlDigital"),
    ]
    
    print(f"Porting {len(missing)} notebooks...")
    print("=" * 80)
    
    success_count = 0
    for example, section in missing:
        print(f"\nProcessing {example}/{section}...")
        output_dir = base_dir / example
        if port_notebook(example, section, ctms_dir, output_dir):
            success_count += 1
    
    print("\n" + "=" * 80)
    print(f"Completed: {success_count}/{len(missing)} notebooks ported successfully")


if __name__ == "__main__":
    main()


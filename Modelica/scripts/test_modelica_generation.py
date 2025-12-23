#!/usr/bin/env python3
"""
Test script to verify generated Modelica files.

This script verifies that all Modelica .mo files are generated correctly
and can be validated using OpenModelica compiler if available.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import List, Tuple


def check_omc_available() -> bool:
    """Check if OpenModelica compiler (omc) is available.
    
    Returns
    -------
    bool
        True if omc is available, False otherwise
    """
    try:
        result = subprocess.run(
            ['omc', '--version'],
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def validate_modelica_file(file_path: Path) -> Tuple[bool, str]:
    """Validate a Modelica file using omc if available.
    
    Parameters
    ----------
    file_path : Path
        Path to the .mo file to validate
        
    Returns
    -------
    Tuple[bool, str]
        (success, message) tuple
    """
    if not check_omc_available():
        return True, "omc not available, skipping validation"
    
    try:
        result = subprocess.run(
            ['omc', '--check', str(file_path)],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0:
            return True, "Validation passed"
        else:
            return False, f"Validation failed: {result.stderr}"
    except subprocess.TimeoutExpired:
        return False, "Validation timed out"
    except Exception as e:
        return False, f"Validation error: {str(e)}"


def check_all_files() -> Tuple[int, int, List[str]]:
    """Check all generated Modelica files.
    
    Returns
    -------
    Tuple[int, int, List[str]]
        (total_files, valid_files, error_messages) tuple
    """
    project_root = Path(__file__).parent.parent.parent
    modelica_dir = project_root / 'Modelica' / 'UMichControls'
    
    systems = [
        'CruiseControl',
        'MotorSpeed',
        'MotorPosition',
        'AircraftPitch',
        'Suspension',
        'InvertedPendulum',
        'BallBeam',
        'Introduction'
    ]
    
    total_files = 0
    valid_files = 0
    errors: List[str] = []
    
    for system in systems:
        system_dir = modelica_dir / system
        
        if system == 'Introduction':
            # Introduction has MassSpringDamper.mo
            model_file = system_dir / 'MassSpringDamper.mo'
        else:
            model_file = system_dir / f'{system}_System.mo'
        
        if not model_file.exists():
            errors.append(f"Missing: {model_file}")
            continue
        
        total_files += 1
        print(f"Checking {model_file}...", end=' ')
        
        # Basic file check
        if model_file.stat().st_size == 0:
            errors.append(f"Empty file: {model_file}")
            print("✗ (empty)")
            continue
        
        # Try to validate with omc if available
        success, message = validate_modelica_file(model_file)
        
        if success:
            valid_files += 1
            print(f"✓ ({message})")
        else:
            errors.append(f"{model_file}: {message}")
            print(f"✗ ({message})")
    
    return total_files, valid_files, errors


def main() -> int:
    """Main entry point for testing.
    
    Returns
    -------
    int
        Exit code (0 for success, 1 for failure)
    """
    print("Testing Modelica file generation...")
    print("=" * 60)
    
    total, valid, errors = check_all_files()
    
    print("=" * 60)
    print(f"Total files: {total}")
    print(f"Valid files: {valid}")
    print(f"Errors: {len(errors)}")
    
    if errors:
        print("\nErrors:")
        for error in errors:
            print(f"  - {error}")
        return 1
    
    if valid == total and total > 0:
        print("\n✓ All files are valid!")
        return 0
    else:
        print("\n✗ Some files failed validation")
        return 1


if __name__ == '__main__':
    sys.exit(main())


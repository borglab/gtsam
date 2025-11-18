#!/usr/bin/env python3
"""
Script to process Jupyter notebooks and add standardized cells:
1. "Open in Colab" button after the title
2. License cell with remove-cell tag
3. Try/except import cell for Colab with remove-cell tag

Usage:
    python standardize_notebooks.py <directory_path>
    
Example:
    python standardize_notebooks.py ../python/gtsam/examples/
"""

import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Any


def create_colab_button_cell(notebook_path: str) -> Dict[str, Any]:
    # Convert absolute path to relative GitHub path
    github_path = notebook_path.replace('/Users/apollo/dev/research/gtsam/', '')
    
    return {
        "cell_type": "markdown",
        "id": "colab_button",
        "metadata": {},
        "source": [
            f'<a href="https://colab.research.google.com/github/borglab/gtsam/blob/develop/{github_path}" target="_parent">'
            '<img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"/></a>'
        ]
    }


def create_license_cell() -> Dict[str, Any]:
    return {
        "cell_type": "markdown",
        "id": "license_cell",
        "metadata": {
            "tags": ["remove-cell"]
        },
        "source": [
            "GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,\n"
            "Atlanta, Georgia 30332-0415\n"
            "All Rights Reserved\n"
            "\n"
            "Authors: Frank Dellaert, et al. (see THANKS for the full author list)\n"
            "\n"
            "See LICENSE for the license information"
        ]
    }


def create_colab_import_cell() -> Dict[str, Any]:
    return {
        "cell_type": "code",
        "execution_count": None,
        "id": "colab_import",
        "metadata": {
            "tags": ["remove-cell"]
        },
        "outputs": [],
        "source": [
            "try:\n"
            "    import google.colab\n"
            "    %pip install --quiet gtsam-develop\n"
            "except ImportError:\n"
            "    pass"
        ]
    }


def has_cell_with_content(cells: List[Dict[str, Any]], content_substring: str) -> bool:
    """Check if any cell contains the given content substring."""
    for cell in cells:
        if "source" in cell:
            source = cell["source"]
            if isinstance(source, list):
                source = "".join(source)
            if content_substring in source:
                return True
    return False


def find_title_cell_index(cells: List[Dict[str, Any]]) -> int:
    """Find the index of the first markdown cell (assumed to be the title)."""
    for i, cell in enumerate(cells):
        if cell.get("cell_type") == "markdown":
            return i
    return 0  # If no markdown cell found, insert at beginning


def process_notebook(notebook_path: Path) -> bool:

    try:
        with open(notebook_path, 'r', encoding='utf-8') as f:
            notebook = json.load(f)
    except (json.JSONDecodeError, FileNotFoundError) as e:
        print(f"Error reading {notebook_path}: {e}")
        return False
    
    cells = notebook.get("cells", [])
    modified = False
    
    # Check what's already present
    has_colab_button = has_cell_with_content(cells, "colab.research.google.com")
    has_license = has_cell_with_content(cells, "GTSAM Copyright")
    has_colab_import = has_cell_with_content(cells, "pip install --quiet gtsam-develop")
    
    # Find where to insert new cells
    title_index = find_title_cell_index(cells)
    insert_index = title_index + 1
    
    cells_to_add = []
    
    if not has_colab_import:
        cells_to_add.append(create_colab_import_cell())
        modified = True
    
    if not has_license:
        cells_to_add.append(create_license_cell())
        modified = True
    
    if not has_colab_button:
        cells_to_add.append(create_colab_button_cell(str(notebook_path)))
        modified = True
    
    for cell in cells_to_add:
        cells.insert(insert_index, cell)
    
    if modified:
        try:
            with open(notebook_path, 'w', encoding='utf-8') as f:
                json.dump(notebook, f, indent=2, ensure_ascii=False)
            print(f"✓ Modified: {notebook_path}")
        except Exception as e:
            print(f"Error writing {notebook_path}: {e}")
            return False
    else:
        print(f"- Skipped: {notebook_path} (already has required cells)")
    
    return modified


def process_directory(directory_path: Path) -> None:

    if not directory_path.exists():
        print(f"Error: Directory {directory_path} does not exist")
        return
    
    if not directory_path.is_dir():
        print(f"Error: {directory_path} is not a directory")
        return
    
    # Find all .ipynb files
    notebook_files = list(directory_path.glob("*.ipynb"))
    
    if not notebook_files:
        print(f"No .ipynb files found in {directory_path}")
        return
    
    print(f"Found {len(notebook_files)} notebook(s) in {directory_path}")
    print("-" * 50)
    
    modified_count = 0
    for notebook_path in sorted(notebook_files):
        if process_notebook(notebook_path):
            modified_count += 1
    
    print("-" * 50)
    print(f"Processing complete: {modified_count}/{len(notebook_files)} notebooks modified")


def main():

    if len(sys.argv) != 2:
        print("Usage: python standardize_notebooks.py <directory_path>")
        print("\nExample:")
        print("    python standardize_notebooks.py ../python/gtsam/examples/")
        sys.exit(1)
    
    directory_path = Path(sys.argv[1]).resolve()
    process_directory(directory_path)


if __name__ == "__main__":
    main()

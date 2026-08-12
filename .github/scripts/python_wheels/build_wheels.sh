#!/bin/bash

# This script calls cibuildwheel to build the wheels for the project. It is used in the build-cibw.yml workflow in .github/workflows.
# Note that the build/python directory contains the wrapper module built for the specified Python version.

set -e
set -x

python -m pip install cibuildwheel
python -m cibuildwheel build/python --output-dir wheelhouse

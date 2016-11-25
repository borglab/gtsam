#!/usr/bin/env bash
# exit if any command returns non-zero
set -e
# print out each command for debugging
set -x

#usage: wrap [--matlab|--cython] absoluteInterfacePath moduleName toolboxPath headerPath
wrap --cython $PWD gtsam $PWD/gtsam ../

python setup.py build_ext --inplace
python -m unittest discover

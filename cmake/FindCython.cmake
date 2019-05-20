# Modifed from: https://github.com/nest/nest-simulator/blob/master/cmake/FindCython.cmake
#
# Find the Cython compiler.
#
# This code sets the following variables:
#
#  CYTHON_FOUND
#  CYTHON_PATH
#  CYTHON_EXECUTABLE
#  CYTHON_VERSION
#
# See also UseCython.cmake

#=============================================================================
# Copyright 2011 Kitware, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#=============================================================================

# Use the Cython executable that lives next to the Python executable
# if it is a local installation.
if(GTSAM_PYTHON_VERSION STREQUAL "Default")
  find_package(PythonInterp)
else()
  find_package(PythonInterp ${GTSAM_PYTHON_VERSION} EXACT)
endif()

if ( PYTHONINTERP_FOUND )
  execute_process( COMMAND "${PYTHON_EXECUTABLE}" "-c"
      "import Cython; print(Cython.__path__[0])"
      RESULT_VARIABLE RESULT
      OUTPUT_VARIABLE CYTHON_PATH
      OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif ()

# RESULT=0 means ok
if ( NOT RESULT )
  get_filename_component( _python_path ${PYTHON_EXECUTABLE} PATH )
  find_program( CYTHON_EXECUTABLE
      NAMES cython cython.bat cython3
      HINTS ${_python_path}
   )
endif ()

# RESULT=0 means ok
if ( NOT RESULT )
  execute_process( COMMAND "${PYTHON_EXECUTABLE}" "-c"
      "import Cython; print(Cython.__version__)"
      RESULT_VARIABLE RESULT
      OUTPUT_VARIABLE CYTHON_VAR_OUTPUT
      ERROR_VARIABLE CYTHON_VAR_OUTPUT
      OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if ( RESULT EQUAL 0 )
    string( REGEX REPLACE ".* ([0-9]+\\.[0-9]+(\\.[0-9]+)?).*" "\\1"
                          CYTHON_VERSION "${CYTHON_VAR_OUTPUT}" )
  endif ()
endif ()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Cython
  FOUND_VAR
    CYTHON_FOUND
  REQUIRED_VARS
    CYTHON_PATH
    CYTHON_EXECUTABLE
  VERSION_VAR
    CYTHON_VERSION
    )


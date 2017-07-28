# Find the cloned version of eigency built and installed by gtsam
#
# This code sets the following variables:
#
#  CLONEDEIGENCY_FOUND
#  CLONEDEIGENCY_INCLUDE_DIRS
#

# Find python
find_package( PythonInterp )
if ( PYTHONINTERP_FOUND )
  execute_process( COMMAND "${PYTHON_EXECUTABLE}" "-c"
    "import clonedEigency; includes=clonedEigency.get_includes(include_eigen=False); print includes[0], ';', includes[1]"
      RESULT_VARIABLE RESULT
      OUTPUT_VARIABLE CLONEDEIGENCY_INCLUDE_DIRS
      OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif ()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args(ClonedEigency
  FOUND_VAR
    CLONEDEIGENCY_FOUND
  REQUIRED_VARS
    CLONEDEIGENCY_INCLUDE_DIRS
)


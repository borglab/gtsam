# Find Eigency
#
# This code sets the following variables:
#
#  EIGENCY_FOUND
#  EIGENCY_INCLUDE_DIRS
#

# Find python
find_package( PythonInterp )
if ( PYTHONINTERP_FOUND )
  execute_process( COMMAND "${PYTHON_EXECUTABLE}" "-c"
    "import eigency; includes=eigency.get_includes(include_eigen=False); print includes[0], ';', includes[1]"
      RESULT_VARIABLE RESULT
      OUTPUT_VARIABLE EIGENCY_INCLUDE_DIRS
      OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif ()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args(Eigency
  FOUND_VAR
    EIGENCY_FOUND
  REQUIRED_VARS
    EIGENCY_INCLUDE_DIRS
)


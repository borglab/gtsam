# Install cython components
include(GtsamCythonWrap)

# Set up building of cython module
set_up_required_cython_packages()

# include eigency headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS "Cythonize and build eigency")
cythonize(cythonize_eigency_core "${CMAKE_CURRENT_SOURCE_DIR}/core.pyx" "core"
  "${PROJECT_BINARY_DIR}/cython/eigency" "${CMAKE_CURRENT_SOURCE_DIR}" "" "")
cythonize(cythonize_eigency_conversions "${CMAKE_CURRENT_SOURCE_DIR}/conversions.pyx" "conversions"
  "${PROJECT_BINARY_DIR}/cython/eigency" "${CMAKE_CURRENT_SOURCE_DIR}" "" "")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION ${GTSAM_CYTHON_INSTALL_PATH})
install(TARGETS cythonize_eigency_core cythonize_eigency_conversions
        DESTINATION ${GTSAM_CYTHON_INSTALL_PATH}/eigency)

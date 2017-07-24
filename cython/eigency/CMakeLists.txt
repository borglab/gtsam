include(GtsamCythonWrap)

# include eigency headers
set(EIGENCY_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGENCY_INCLUDE_DIR})

# cythonize and build eigency
message(STATUS "Cythonize and build eigency")
cythonize(cythonize_eigency_core "${CMAKE_CURRENT_SOURCE_DIR}/core.pyx" "core"
  "${PROJECT_BINARY_DIR}/cython/eigency" "${EIGENCY_INCLUDE_DIR}" "" "")
cythonize(cythonize_eigency_conversions "${CMAKE_CURRENT_SOURCE_DIR}/conversions.pyx" "conversions"
  "${PROJECT_BINARY_DIR}/cython/eigency" "${EIGENCY_INCLUDE_DIR}" "" "")

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION ${GTSAM_CYTHON_INSTALL_PATH})
install(TARGETS cythonize_eigency_core cythonize_eigency_conversions
        DESTINATION ${GTSAM_CYTHON_INSTALL_PATH}/eigency)
configure_file(__init__.py.in ${GTSAM_CYTHON_INSTALL_PATH}/eigency/__init__.py)

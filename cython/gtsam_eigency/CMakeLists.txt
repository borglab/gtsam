include(GtsamCythonWrap)

# Copy eigency's sources to the build folder
# so that the cython-generated header "conversions_api.h" can be found when cythonizing eigency's core
# and eigency's cython pxd headers can be found when cythonizing gtsam
file(COPY "." DESTINATION ".")
set(OUTPUT_DIR "${GTSAM_CYTHON_INSTALL_PATH}/gtsam_eigency")
set(EIGENCY_INCLUDE_DIR ${OUTPUT_DIR})

# This is to make the build/cython/gtsam_eigency folder a python package
configure_file(__init__.py.in ${OUTPUT_DIR}/__init__.py)

# include eigency headers
include_directories(${EIGENCY_INCLUDE_DIR})

# Cythonize and build eigency
message(STATUS "Cythonize and build eigency")
# Important trick: use "../gtsam_eigency/conversions.pyx" to let cython know that the conversions module is
# a part of the gtsam_eigency package and generate the function call import_gtsam_eigency__conversions()
# in conversions_api.h correctly!
cythonize(cythonize_eigency_conversions "../gtsam_eigency/conversions.pyx" "conversions"
  "${OUTPUT_DIR}" "${EIGENCY_INCLUDE_DIR}" "" "" "")
cythonize(cythonize_eigency_core "../gtsam_eigency/core.pyx" "core"
  ${OUTPUT_DIR} "${EIGENCY_INCLUDE_DIR}" "" "" "")

# Include Eigen headers:
target_include_directories(cythonize_eigency_conversions PUBLIC
  $<BUILD_INTERFACE:${GTSAM_EIGEN_INCLUDE_FOR_BUILD}>
  $<INSTALL_INTERFACE:${GTSAM_EIGEN_INCLUDE_FOR_INSTALL}>
)
target_include_directories(cythonize_eigency_core PUBLIC
  $<BUILD_INTERFACE:${GTSAM_EIGEN_INCLUDE_FOR_BUILD}>
  $<INSTALL_INTERFACE:${GTSAM_EIGEN_INCLUDE_FOR_INSTALL}>
)

add_dependencies(cythonize_eigency_core cythonize_eigency_conversions)
add_custom_target(cythonize_eigency)
add_dependencies(cythonize_eigency cythonize_eigency_conversions cythonize_eigency_core)

if(TARGET ${python_install_target})
  add_dependencies(${python_install_target} cythonize_eigency)
endif()

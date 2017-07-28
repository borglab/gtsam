include(GtsamCythonWrap)

# Copy eigency's sources to the build folder
# so that the cython-generated header "conversions_api.h" can be found when cythonizing eigency's core
# and eigency's cython pxd headers can be found when cythonizing gtsam
file(COPY "." DESTINATION ".")
set(OUTPUT_DIR "${PROJECT_BINARY_DIR}/cython/clonedEigency")
set(EIGENCY_INCLUDE_DIR ${OUTPUT_DIR})

# This is to make the build/cython/clonedEigency folder a python package
configure_file(__init__.py.in ${PROJECT_BINARY_DIR}/cython/clonedEigency/__init__.py)

# include eigency headers
include_directories(${EIGENCY_INCLUDE_DIR})

# Cythonize and build eigency
message(STATUS "Cythonize and build eigency")
# Important trick: use "../clonedEigency/conversions.pyx" to let cython know that the conversions module is
# a part of the clonedEigency package and generate the function call import_clonedEigency__conversions()
# in conversions_api.h correctly!!!
cythonize(cythonize_eigency_conversions "../clonedEigency/conversions.pyx" "conversions"
  "${OUTPUT_DIR}" "${EIGENCY_INCLUDE_DIR}" "" "")
cythonize(cythonize_eigency_core "../clonedEigency/core.pyx" "core"
  ${OUTPUT_DIR} "${EIGENCY_INCLUDE_DIR}" "" "")
add_dependencies(cythonize_eigency_core cythonize_eigency_conversions)
add_custom_target(cythonize_eigency)
add_dependencies(cythonize_eigency cythonize_eigency_conversions cythonize_eigency_core)

# install
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION ${GTSAM_CYTHON_INSTALL_PATH})
install(TARGETS cythonize_eigency_core cythonize_eigency_conversions
        DESTINATION "${GTSAM_CYTHON_INSTALL_PATH}/clonedEigency")
install(FILES "${OUTPUT_DIR}/conversions_api.h" DESTINATION ${GTSAM_CYTHON_INSTALL_PATH}/clonedEigency)
configure_file(__init__.py.in ${GTSAM_CYTHON_INSTALL_PATH}/clonedEigency/__init__.py)

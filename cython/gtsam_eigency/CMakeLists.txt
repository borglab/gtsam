include(GtsamCythonWrap)

# Copy eigency's sources to the build folder
# so that the cython-generated header "conversions_api.h" can be found when cythonizing eigency's core
# and eigency's cython pxd headers can be found when cythonizing gtsam
file(COPY "." DESTINATION ".")
set(OUTPUT_DIR "${PROJECT_BINARY_DIR}/cython/gtsam_eigency")
set(EIGENCY_INCLUDE_DIR ${OUTPUT_DIR})

# This is to make the build/cython/gtsam_eigency folder a python package
configure_file(__init__.py.in ${PROJECT_BINARY_DIR}/cython/gtsam_eigency/__init__.py)

# include eigency headers
include_directories(${EIGENCY_INCLUDE_DIR})

# Cythonize and build eigency
message(STATUS "Cythonize and build eigency")
# Important trick: use "../gtsam_eigency/conversions.pyx" to let cython know that the conversions module is
# a part of the gtsam_eigency package and generate the function call import_gtsam_igency__conversions()
# in conversions_api.h correctly!!!
cythonize(cythonize_eigency_conversions "../gtsam_eigency/conversions.pyx" "conversions"
  "${OUTPUT_DIR}" "${EIGENCY_INCLUDE_DIR}" "" "" "")
cythonize(cythonize_eigency_core "../gtsam_eigency/core.pyx" "core"
  ${OUTPUT_DIR} "${EIGENCY_INCLUDE_DIR}" "" "" "")
add_dependencies(cythonize_eigency_core cythonize_eigency_conversions)
add_custom_target(cythonize_eigency)
add_dependencies(cythonize_eigency cythonize_eigency_conversions cythonize_eigency_core)

# install
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DESTINATION ${GTSAM_CYTHON_INSTALL_PATH}
        PATTERN "CMakeLists.txt" EXCLUDE
        PATTERN "__init__.py.in" EXCLUDE)
install(TARGETS cythonize_eigency_core cythonize_eigency_conversions
        DESTINATION "${GTSAM_CYTHON_INSTALL_PATH}/gtsam_eigency")
install(FILES ${OUTPUT_DIR}/conversions_api.h DESTINATION ${GTSAM_CYTHON_INSTALL_PATH}/gtsam_eigency)
configure_file(__init__.py.in ${OUTPUT_DIR}/__init__.py)
install(FILES ${OUTPUT_DIR}/__init__.py DESTINATION ${GTSAM_CYTHON_INSTALL_PATH}/gtsam_eigency)

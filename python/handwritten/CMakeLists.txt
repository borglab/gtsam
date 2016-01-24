# get subdirectories list
subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

# get the sources needed to compile gtsam python module
set(gtsam_python_srcs "")
foreach(subdir ${SUBDIRS})
	file(GLOB ${subdir}_src "${subdir}/*.cpp")
	list(APPEND gtsam_python_srcs ${${subdir}_src})
endforeach()

# Create the library
add_library(gtsam_python SHARED exportgtsam.cpp ${gtsam_python_srcs})
set_target_properties(gtsam_python PROPERTIES
	OUTPUT_NAME         gtsam_python
	SKIP_BUILD_RPATH    TRUE
	CLEAN_DIRECT_OUTPUT 1
)
target_link_libraries(gtsam_python
                      ${Boost_PYTHON${BOOST_PYTHON_VERSION_SUFFIX_UPPERCASE}_LIBRARY}
                      ${PYTHON_LIBRARY} gtsam)

# Cause the library to be output in the correct directory.
# TODO: Change below to work on different systems (currently works only with Linux)
add_custom_command(
	OUTPUT ${CMAKE_BINARY_DIR}/python/gtsam/_libgtsam_python.so
	DEPENDS gtsam_python
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gtsam_python> ${CMAKE_BINARY_DIR}/python/gtsam/_libgtsam_python.so
	COMMENT "Copying extension module to python/gtsam/_libgtsam_python.so"
)
add_custom_target(copy_gtsam_python_module ALL DEPENDS ${CMAKE_BINARY_DIR}/python/gtsam/_libgtsam_python.so)
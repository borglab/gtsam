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
string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type_toupper)
set_target_properties(gtsam_python PROPERTIES
	OUTPUT_NAME                   gtsampy
    PREFIX                        ""
    ${build_type_toupper}_POSTFIX ""
	SKIP_BUILD_RPATH              TRUE
	CLEAN_DIRECT_OUTPUT           1
)

target_include_directories(gtsam_python SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(gtsam_python SYSTEM PUBLIC ${NUMPY_INCLUDE_DIRS})
target_include_directories(gtsam_python SYSTEM PUBLIC ${PYTHON_INCLUDE_DIRS})
target_include_directories(gtsam_python SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(gtsam_python SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include/)

target_link_libraries(gtsam_python
                      ${Boost_PYTHON${BOOST_PYTHON_VERSION_SUFFIX_UPPERCASE}_LIBRARY}
                      ${PYTHON_LIBRARY} gtsam)

# Cause the library to be output in the correct directory.
# TODO: Change below to work on different systems (currently works only with Linux)
set(output_path ${CMAKE_CURRENT_BINARY_DIR}/../gtsam/_libgtsam_python.so)
add_custom_command(
	OUTPUT ${output_path}
	DEPENDS gtsam_python
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gtsam_python> ${output_path}
	COMMENT "Copying extension module to python/gtsam/_libgtsam_python.so"
)
add_custom_target(copy_gtsam_python_module ALL DEPENDS ${output_path})
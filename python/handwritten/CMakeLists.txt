
# get subdirectories list
subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

# get the sources needed to compile gtsam python module
set(gtsam_python_srcs "")
foreach(subdir ${SUBDIRS})
	file(GLOB ${subdir}_src "${subdir}/*.cpp")
	list(APPEND gtsam_python_srcs ${${subdir}_src})
endforeach()

# Create the library
set(moduleName gtsam)
set(gtsamLib gtsam)
add_library(${moduleName}_python SHARED exportgtsam.cpp ${gtsam_python_srcs})

set_target_properties(${moduleName}_python PROPERTIES
    OUTPUT_NAME         ${moduleName}_python
    SKIP_BUILD_RPATH TRUE
    CLEAN_DIRECT_OUTPUT 1)

target_link_libraries(${moduleName}_python ${Boost_PYTHON${BOOST_PYTHON_VERSION_SUFFIX_UPPERCASE}_LIBRARY} ${PYTHON_LIBRARY} ${gtsamLib}) #temp

# Cause the library to be output in the correct directory.
add_custom_command(TARGET ${moduleName}_python
	POST_BUILD
	COMMAND cp -v $<TARGET_FILE:${moduleName}_python> ${CMAKE_BINARY_DIR}/python/gtsam/_$<TARGET_FILE_NAME:${moduleName}_python>
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	COMMENT "Copying library files to python directory" )

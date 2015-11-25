# Macro to get list of subdirectories
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

# get subdirectories list
SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

# get the sources needed to compile gtsam python module
SET(gtsam_python_srcs "")
FOREACH(subdir ${SUBDIRS})
	file(GLOB ${subdir}_src "${subdir}/*.cpp")
	LIST(APPEND gtsam_python_srcs ${${subdir}_src})
ENDFOREACH()

# Create the library
set(moduleName gtsam)
set(gtsamLib gtsam)
add_library(${moduleName}_python SHARED exportgtsam.cpp ${gtsam_python_srcs})

set_target_properties(${moduleName}_python PROPERTIES
    OUTPUT_NAME         ${moduleName}_python
    CLEAN_DIRECT_OUTPUT 1)

target_link_libraries(${moduleName}_python ${Boost_PYTHON_LIBRARY} ${PYTHON_LIBRARY} ${gtsamLib}) #temp

# On OSX and Linux, the python library must end in the extension .so. Build this
# filename here.
get_property(PYLIB_OUTPUT_FILE TARGET ${moduleName}_python PROPERTY LOCATION)
set(PYLIB_OUTPUT_FILE $<TARGET_FILE:${moduleName}_python>)
get_filename_component(PYLIB_OUTPUT_NAME ${PYLIB_OUTPUT_FILE} NAME_WE)
set(PYLIB_SO_NAME lib${moduleName}_python.so)

# Installs the library in the gtsam folder, which is used by setup.py to create the gtsam package
set(PYTHON_MODULE_DIRECTORY ${CMAKE_SOURCE_DIR}/python/gtsam)
# Cause the library to be output in the correct directory.
add_custom_command(TARGET ${moduleName}_python
	POST_BUILD
	COMMAND cp -v ${PYLIB_OUTPUT_FILE} ${PYTHON_MODULE_DIRECTORY}/_${PYLIB_SO_NAME}
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	COMMENT "Copying library files to python directory" )

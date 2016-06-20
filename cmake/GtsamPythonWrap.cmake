#Setup cache options
set(GTSAM_PYTHON_VERSION "Default" CACHE STRING "Target python version for GTSAM python module. Use 'Default' to chose the default version")
set(GTSAM_BUILD_PYTHON_FLAGS "" CACHE STRING "Extra flags for running Matlab PYTHON compilation")
set(GTSAM_PYTHON_INSTALL_PATH "" CACHE PATH "Python toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/borg/python")
if(NOT GTSAM_PYTHON_INSTALL_PATH)
  set(GTSAM_PYTHON_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/borg/python")
endif() 

#Author: Paul Furgale Modified by Andrew Melim
function(wrap_python TARGET_NAME PYTHON_MODULE_DIRECTORY)
  # # Boost
  # find_package(Boost COMPONENTS python filesystem system REQUIRED)
  # include_directories(${Boost_INCLUDE_DIRS})

  # # Find Python
  # FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
  # INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})

  IF(APPLE)
    # The apple framework headers don't include the numpy headers for some reason.
    GET_FILENAME_COMPONENT(REAL_PYTHON_INCLUDE ${PYTHON_INCLUDE_DIRS} REALPATH)
    IF( ${REAL_PYTHON_INCLUDE} MATCHES Python.framework)
      message("Trying to find extra headers for numpy from ${REAL_PYTHON_INCLUDE}.")
      message("Looking in ${REAL_PYTHON_INCLUDE}/../../Extras/lib/python/numpy/core/include/numpy")
      FIND_PATH(NUMPY_INCLUDE_DIR arrayobject.h
  ${REAL_PYTHON_INCLUDE}/../../Extras/lib/python/numpy/core/include/numpy
  ${REAL_PYTHON_INCLUDE}/numpy
  )
      IF(${NUMPY_INCLUDE_DIR} MATCHES NOTFOUND)
  message("Unable to find numpy include directories: ${NUMPY_INCLUDE_DIR}")
      ELSE()
  message("Found headers at ${NUMPY_INCLUDE_DIR}")
  INCLUDE_DIRECTORIES(${NUMPY_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${NUMPY_INCLUDE_DIR}/..)
      ENDIF()
    ENDIF()
  ENDIF(APPLE)

  if(MSVC)
    add_library(${moduleName}_python MODULE ${ARGN})
    set_target_properties(${moduleName}_python PROPERTIES
        OUTPUT_NAME         ${moduleName}_python
        CLEAN_DIRECT_OUTPUT 1
        VERSION             1
        SOVERSION           0
        SUFFIX              ".pyd")
      target_link_libraries(${moduleName}_python ${Boost_PYTHON_LIBRARY} ${PYTHON_LIBRARY} ${gtsamLib}) #temp

    set(PYLIB_OUTPUT_FILE $<TARGET_FILE:${moduleName}_python>)
    message(${PYLIB_OUTPUT_FILE})
    get_filename_component(PYLIB_OUTPUT_NAME ${PYLIB_OUTPUT_FILE} NAME_WE)
    set(PYLIB_SO_NAME ${PYLIB_OUTPUT_NAME}.pyd)

  ELSE()
    # Create a shared library
    add_library(${moduleName}_python SHARED ${generated_cpp_file})

    set_target_properties(${moduleName}_python PROPERTIES
        OUTPUT_NAME         ${moduleName}_python
        CLEAN_DIRECT_OUTPUT 1)
      target_link_libraries(${moduleName}_python ${Boost_PYTHON_LIBRARY} ${PYTHON_LIBRARY} ${gtsamLib}) #temp
    # On OSX and Linux, the python library must end in the extension .so. Build this
    # filename here.
    get_property(PYLIB_OUTPUT_FILE TARGET ${moduleName}_python PROPERTY LOCATION)
    set(PYLIB_OUTPUT_FILE $<TARGET_FILE:${moduleName}_python>)
    message(${PYLIB_OUTPUT_FILE})
    get_filename_component(PYLIB_OUTPUT_NAME ${PYLIB_OUTPUT_FILE} NAME_WE)
    set(PYLIB_SO_NAME lib${moduleName}_python.so)
  ENDIF(MSVC)

  # Installs the library in the gtsam folder, which is used by setup.py to create the gtsam package
  set(PYTHON_MODULE_DIRECTORY ${CMAKE_SOURCE_DIR}/python/gtsam)
  # Cause the library to be output in the correct directory.
  add_custom_command(TARGET ${moduleName}_python
    POST_BUILD
    COMMAND cp -v ${PYLIB_OUTPUT_FILE} ${PYTHON_MODULE_DIRECTORY}/${PYLIB_SO_NAME}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Copying library files to python directory" )

  # Cause the library to be output in the correct directory.
  add_custom_command(TARGET ${TARGET_NAME}
    POST_BUILD
    COMMAND cp -v ${PYLIB_OUTPUT_FILE} ${PYTHON_MODULE_DIRECTORY}/${PYLIB_SO_NAME}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Copying library files to python directory" )

  get_directory_property(AMCF ADDITIONAL_MAKE_CLEAN_FILES)
  list(APPEND AMCF ${PYTHON_MODULE_DIRECTORY}/${PYLIB_SO_NAME})
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${AMCF}") 
endfunction(wrap_python)

# Macro to get list of subdirectories
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
        list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

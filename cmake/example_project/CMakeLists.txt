# This file should be used as a template for creating new projects using the CMake tools
# This project has the following features
#    - GTSAM linking
#    - Boost linking
#    - Unit tests via CppUnitLite
#    - Automatic detection of sources and headers in subfolders
#    - Installation of library and headers
#    - Matlab wrap interface with within-project building
#    - Use of GTSAM cmake macros

###################################################################################
# To create your own project, replace "myproject" with the actual name of your project
cmake_minimum_required(VERSION 2.6)
enable_testing()
project(myproject CXX C)               

# Add the cmake subfolder to the cmake module path - necessary to use macros
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/cmake")

# Ensure that local folder is searched before library folders
include_directories(BEFORE "${CMAKE_SOURCE_DIR}")

# Load build type flags and default to Debug mode
include(GtsamBuildTypes)

###################################################################################
# Create a list of library dependencies 
# These will be linked with executables
set(library_deps "")
set(linking_mode "static")

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
list(APPEND library_deps gtsam-${linking_mode} gtsam_unstable-${linking_mode})

# Include ransac
find_package(ransac REQUIRED) # Uses installed package
list(APPEND library_deps ransac-${linking_mode})

# Boost - same requirement as gtsam
find_package(Boost 1.43 COMPONENTS 
    serialization 
    system 
    filesystem 
    thread 
    date_time 
    REQUIRED)
list(APPEND library_deps 
    ${Boost_SERIALIZATION_LIBRARY} 
    ${Boost_SYSTEM_LIBRARY} 
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY} 
    ${Boost_DATE_TIME_LIBRARY})

include_directories(${Boost_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR} ${ransac_INCLUDE_DIR})

###################################################################################
# List subdirs to process tests/sources
# Each of these will be scanned for new files
set (myproject_subdirs 
    "."   # ensure root folder gets included
    stuff 
    things
    )

# loop through subdirs to install and build up source lists
set(myproject_lib_source "")
set(myproject_tests_source "")
set(myproject_scripts_source "")
foreach(subdir ${myproject_subdirs})
    # Installing headers
    message(STATUS "Installing ${subdir}")
    file(GLOB sub_myproject_headers "myproject/${subdir}/*.h")
    install(FILES ${sub_myproject_headers} DESTINATION include/myproject/${subdir})
    
    # add sources to main sources list
    file(GLOB subdir_srcs "myproject/${subdir}/*.cpp")
    list(APPEND myproject_lib_source ${subdir_srcs})
    
    # add tests to main tests list
    file(GLOB subdir_test_srcs "myproject/${subdir}/tests/*.cpp")
    list(APPEND myproject_tests_source ${subdir_test_srcs})
    
    # add scripts to main tests list
    file(GLOB subdir_scripts_srcs "myproject/${subdir}/scripts/*.cpp")
    list(APPEND myproject_scripts_source ${subdir_scripts_srcs})
endforeach(subdir)

set(myproject_version   ${myproject_VERSION_MAJOR}.${myproject_VERSION_MINOR}.${myproject_VERSION_PATCH})
set(myproject_soversion ${myproject_VERSION_MAJOR})
message(STATUS "GTSAM Version: ${gtsam_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# Build library (static and shared versions)
# Include installed versions
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_library(${PROJECT_NAME}-shared SHARED ${myproject_lib_source})
set_target_properties(${PROJECT_NAME}-shared PROPERTIES 
    OUTPUT_NAME         ${PROJECT_NAME}
    CLEAN_DIRECT_OUTPUT 1)
install(TARGETS myproject-shared EXPORT myproject-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
list(APPEND myproject_EXPORTED_TARGETS myproject-shared)

add_library(${PROJECT_NAME}-static STATIC ${myproject_lib_source})
set_target_properties(${PROJECT_NAME}-static PROPERTIES 
    OUTPUT_NAME         ${PROJECT_NAME}
    CLEAN_DIRECT_OUTPUT 1)
install(TARGETS myproject-static EXPORT myproject-exports ARCHIVE DESTINATION lib)
list(APPEND myproject_EXPORTED_TARGETS myproject-static)

install(TARGETS ${PROJECT_NAME}-shared LIBRARY DESTINATION lib )
install(TARGETS ${PROJECT_NAME}-static ARCHIVE DESTINATION lib )

# Disabled tests - subtract these from the test files
# Note the need for a full path
set(disabled_tests 
    "dummy"
    #"${CMAKE_SOURCE_DIR}/myproject/geometry/tests/testCovarianceEllipse.cpp"
)
list(REMOVE_ITEM myproject_tests_source ${disabled_tests})

###################################################################################
# Build tests
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
foreach(test_src_file ${myproject_tests_source})
    get_filename_component(test_base ${test_src_file} NAME_WE)
    message(STATUS "Adding test ${test_src_file} with base name ${test_base}" )
    add_executable(${test_base} ${test_src_file})
    target_link_libraries(${test_base} ${PROJECT_NAME}-${linking_mode} ${library_deps} CppUnitLite)
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check ${test_base})
endforeach(test_src_file)

# Build scripts
foreach(script_src_file ${myproject_scripts_source})
    get_filename_component(script_base ${script_src_file} NAME_WE)
    message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
    add_executable(${script_base} ${script_src_file})
    target_link_libraries(${script_base} ${PROJECT_NAME}-${linking_mode} ${library_deps} CppUnitLite)
    add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)

###################################################################################
# Matlab wrapping
include(GtsamMatlabWrap)
set(MEX_COMMAND "mex" CACHE STRING "Command to use for executing mex (if on path, 'mex' will work)")
set(GTSAM_BUILD_MEX_BINARY_FLAGS "" CACHE STRING "Extra flags for running Matlab MEX compilation")
set(MYPROJECT_TOOLBOX_DIR "../matlab/myproject" CACHE PATH "Install folder for matlab toolbox - defaults to inside project")
set(WRAP_HEADER_PATH "${GTSAM_DIR}/../../../include")
set(MYPROJECT_TOOLBOX_FLAGS 
    ${GTSAM_BUILD_MEX_BINARY_FLAGS} -I${PROJECT_SOURCE_DIR} -I${PROJECT_SOURCE_DIR}/myproject -I${Boost_INCLUDE_DIR} -I${MEX_INCLUDE_ROOT} -I${GTSAM_INCLUDE_DIR} -I${WRAP_HEADER_PATH} -Wl,-rpath,${CMAKE_BINARY_DIR}:${CMAKE_INSTALL_PREFIX}/lib)
set(MYPROJECT_LIBRARY_DEPS gtsam gtsam_unstable ransac myproject)
set(GTSAM_BUILD_MEX_BIN ON)
 
# Function to setup codegen, building and installation of the wrap toolbox
# This wrap setup function assumes that the toolbox will be installed directly, 
# with predictable matlab.h sourcing
# params:
#  moduleName       : the name of the module, interface file must be called moduleName.h
#  mexFlags         : Compilation flags to be passed to the mex compiler 
#  modulePath       : relative path to module markup header file (called moduleName.h)
#  otherLibraries   : list of library targets this should depend on
#  toolboxPath      : the directory in which to generate/build wrappers
#  wrap_header_path : path to the installed wrap header  
wrap_library_generic(myproject "${MYPROJECT_TOOLBOX_FLAGS}" "" "${MYPROJECT_LIBRARY_DEPS}" "${MYPROJECT_TOOLBOX_DIR}" "${WRAP_HEADER_PATH}")

###################################################################################
# Create Install config and export files
# This config file takes the place of FindXXX.cmake scripts
include(GtsamMakeConfigFile)
GtsamMakeConfigFile(myproject)
export(TARGETS ${myproject_EXPORTED_TARGETS} FILE myproject-exports.cmake)
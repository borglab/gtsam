# Build/install Wrap

# Build the executable itself
file(GLOB wrap_srcs "*.cpp")
list(REMOVE_ITEM wrap_srcs wrap.cpp)
add_library(wrap_lib STATIC ${wrap_srcs})
add_executable(wrap wrap.cpp)
target_link_libraries(wrap wrap_lib)

# Install wrap binary
option(GTSAM_INSTALL_WRAP "Enable/Disable installation of wrap utility" ON)
if (GTSAM_INSTALL_WRAP)
    install(TARGETS wrap DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif(GTSAM_INSTALL_WRAP)

# Install matlab header
install(FILES matlab.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/wrap)

if (GTSAM_LINK_BINARIES_AGAINST_CONVENIENCE_LIBS)
    set(convenience_libs
       base
       )
else (GTSAM_LINK_BINARIES_AGAINST_CONVENIENCE_LIBS)
    set(convenience_libs
       gtsam-static)
endif (GTSAM_LINK_BINARIES_AGAINST_CONVENIENCE_LIBS)

# Build tests
if (GTSAM_BUILD_TESTS)
    add_custom_target(check.wrap COMMAND ${CMAKE_CTEST_COMMAND})
    file(GLOB wrap_test_srcs "tests/test*.cpp")
    add_definitions(-DTOPSRCDIR="${CMAKE_SOURCE_DIR}")
    foreach(test_src ${wrap_test_srcs} )
        get_filename_component(test_base ${test_src} NAME_WE)
        set( test_bin wrap.${test_base} )
        add_executable(${test_bin} EXCLUDE_FROM_ALL ${test_src})
        add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
        add_dependencies(check ${test_bin})
        add_dependencies(check.wrap ${test_bin})
        target_link_libraries(${test_bin} CppUnitLite ${convenience_libs} wrap_lib)
        add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
    endforeach(test_src)
endif(GTSAM_BUILD_TESTS)

# Wrap codegen
#usage: wrap mexExtension interfacePath moduleName toolboxPath
#  mexExtension  : OS/CPU-dependent extension for MEX binaries
#  interfacePath : *absolute* path to directory of module interface file
#  moduleName    : the name of the module, interface file must be called moduleName.h
#  toolboxPath   : the directory in which to generate the wrappers
#  [mexFlags]    : extra flags for the mex command

set(mexFlags "-I${Boost_INCLUDE_DIR} -I${CMAKE_INSTALL_PREFIX}/include -I${CMAKE_INSTALL_PREFIX}/include/gtsam -I${CMAKE_INSTALL_PREFIX}/include/gtsam/base -I${CMAKE_INSTALL_PREFIX}/include/gtsam/geometry -I${CMAKE_INSTALL_PREFIX}/include/gtsam/linear -I${CMAKE_INSTALL_PREFIX}/include/gtsam/nonlinear -I${CMAKE_INSTALL_PREFIX}/include/gtsam/slam -L${CMAKE_INSTALL_PREFIX}/lib -lgtsam")
set(toolbox_path ${CMAKE_BINARY_DIR}/wrap/gtsam)
set(moduleName gtsam)

## Determine the mex extension
# Apple Macintosh (64-bit) mexmaci64
# Linux (32-bit) mexglx
# Linux (64-bit) mexa64
# Microsoft Windows (32-bit) mexw32
# Windows (64-bit) mexw64

# only support 64-bit apple
if(CMAKE_HOST_APPLE)
    set(GTSAM_MEX_BIN_EXTENSION_default mexmaci64)
endif(CMAKE_HOST_APPLE)

if(NOT CMAKE_HOST_APPLE)
    # check 64 bit
    if( ${CMAKE_SIZEOF_VOID_P} EQUAL 4 )
      set( HAVE_64_BIT 0 )
    endif( ${CMAKE_SIZEOF_VOID_P} EQUAL 4 )
    
    if( ${CMAKE_SIZEOF_VOID_P} EQUAL 8 )
      set( HAVE_64_BIT 1 )
    endif( ${CMAKE_SIZEOF_VOID_P} EQUAL 8 )
    
    # Check for linux machines
    if (CMAKE_HOST_UNIX)
        if (HAVE_64_BIT)
            set(GTSAM_MEX_BIN_EXTENSION_default mexa64)
        else (HAVE_64_BIT)
            set(GTSAM_MEX_BIN_EXTENSION_default mexglx)
        endif (HAVE_64_BIT)
    endif(CMAKE_HOST_UNIX)
    
    # Check for windows machines
    if (CMAKE_HOST_WIN32)
        if (HAVE_64_BIT)
            set(GTSAM_MEX_BIN_EXTENSION_default mexw64)
        else (HAVE_64_BIT)
            set(GTSAM_MEX_BIN_EXTENSION_default mexw32)
        endif (HAVE_64_BIT)
    endif(CMAKE_HOST_WIN32)
endif(NOT CMAKE_HOST_APPLE)

# Allow for setting mex extension manually
set(GTSAM_MEX_BIN_EXTENSION ${GTSAM_MEX_BIN_EXTENSION_default} CACHE DOCSTRING "Extension for matlab mex files")
message(STATUS "Detected Matlab mex extension: ${GTSAM_MEX_BIN_EXTENSION_default}")
message(STATUS "Current Matlab mex extension: ${GTSAM_MEX_BIN_EXTENSION}")

# Actual build commands - separated by OS
add_custom_target(wrap_gtsam ALL COMMAND 
    ./wrap ${GTSAM_MEX_BIN_EXTENSION} ${CMAKE_SOURCE_DIR} ${moduleName} ${toolbox_path} "${mexFlags}"
    DEPENDS wrap)

option(GTSAM_INSTALL_MATLAB_TOOLBOX  "Enable/Disable installation of matlab toolbox"  ON)
option(GTSAM_INSTALL_MATLAB_EXAMPLES "Enable/Disable installation of matlab examples" ON)
option(GTSAM_INSTALL_MATLAB_TESTS    "Enable/Disable installation of matlab tests"    ON)

set(GTSAM_TOOLBOX_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/borg/toolbox CACHE DOCSTRING "Path to install matlab toolbox")

if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Primary toolbox files
    message(STATUS "Installing Matlab Toolbox to ${GTSAM_TOOLBOX_INSTALL_PATH}")
    install(DIRECTORY DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}) # make an empty folder
    # exploit need for trailing slash to specify a full folder, rather than just its contents to copy
    install(DIRECTORY ${toolbox_path} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH})
    
    # Examples
    if (GTSAM_INSTALL_MATLAB_EXAMPLES)
        message(STATUS "Installing Matlab Toolbox Examples")
        file(GLOB matlab_examples "${CMAKE_SOURCE_DIR}/examples/matlab/*.m")
        install(FILES ${matlab_examples} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam/examples)
    endif (GTSAM_INSTALL_MATLAB_EXAMPLES)
    
    # Tests
    if (GTSAM_INSTALL_MATLAB_TESTS)
        message(STATUS "Installing Matlab Toolbox Tests")
        file(GLOB matlab_tests "${CMAKE_SOURCE_DIR}/tests/matlab/*.m")
        install(FILES ${matlab_tests} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam/tests)
    endif (GTSAM_INSTALL_MATLAB_TESTS)
endif (GTSAM_INSTALL_MATLAB_TOOLBOX)

# Build/install Wrap

# Build the executable itself
file(GLOB wrap_srcs "*.cpp")
list(REMOVE_ITEM wrap_srcs wrap.cpp)
add_library(wrap_lib STATIC ${wrap_srcs})
add_executable(wrap wrap.cpp)
target_link_libraries(wrap wrap_lib)

# Install wrap binary
install(TARGETS wrap DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# Install matlab header
install(FILES matlab.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/wrap)

# Build tests
add_custom_target(check.wrap COMMAND ${CMAKE_CTEST_COMMAND})

# Build tests
file(GLOB wrap_test_srcs "tests/test*.cpp")
add_definitions(-DTOPSRCDIR="${CMAKE_SOURCE_DIR}")
foreach(test_src ${wrap_test_srcs} )
    get_filename_component(test_base ${test_src} NAME_WE)
    set( test_bin wrap.${test_base} )
    add_executable(${test_bin} EXCLUDE_FROM_ALL ${test_src})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
    add_dependencies(check ${test_bin})
    add_dependencies(check.wrap ${test_bin})
    target_link_libraries(${test_bin} CppUnitLite gtsam-static wrap_lib)
    add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
endforeach(test_src)

# Wrap codegen
#usage: wrap mexExtension interfacePath moduleName toolboxPath
#  mexExtension  : OS/CPU-dependent extension for MEX binaries
#  interfacePath : *absolute* path to directory of module interface file
#  moduleName    : the name of the module, interface file must be called moduleName.h
#  toolboxPath   : the directory in which to generate the wrappers
#  [mexFlags]    : extra flags for the mex command

set(gtsam_matlab_toolbox ${CMAKE_INSTALL_PREFIX}/borg/toolbox)
set(mexFlags "${Boost_INCLUDE_DIR} -I${CMAKE_INSTALL_PREFIX}/include -I${CMAKE_INSTALL_PREFIX}/include/gtsam -I${CMAKE_INSTALL_PREFIX}/include/gtsam/base -I${CMAKE_INSTALL_PREFIX}/include/gtsam/geometry -I${CMAKE_INSTALL_PREFIX}/include/gtsam/linear -I${CMAKE_INSTALL_PREFIX}/include/gtsam/nonlinear -I${CMAKE_INSTALL_PREFIX}/include/gtsam/slam -L${CMAKE_BINARY_DIR} -lgtsam")
set(toolbox_path ${CMAKE_BINARY_DIR}/wrap/gtsam)
set(moduleName gtsam)

# Actual build commands - separated by OS
# FIXME: use mexext utility or flags to set this variable correctly
add_custom_target(wrap_gtsam COMMAND 
    ./wrap mexa64 ${CMAKE_SOURCE_DIR} ${moduleName} ${toolbox_path} "${mexFlags}")
add_custom_target(wrapmac_gtsam COMMAND 
    ./wrap mexmaci64 ${CMAKE_SOURCE_DIR} ${moduleName} ${toolbox_path} "${mexFlags}")

# Install toolbox 
# FIXME: parameterize this install path
set(toolbox_install_path ${CMAKE_INSTALL_PREFIX}/borg/toolbox)

# Primary toolbox files
install(DIRECTORY ${toolbox_path} DESTINATION ${toolbox_install_path} FILES_MATCHING PATTERN "*.m")
install(DIRECTORY ${toolbox_path} DESTINATION ${toolbox_install_path} FILES_MATCHING PATTERN "*.cpp")
install(DIRECTORY ${toolbox_path} DESTINATION ${toolbox_install_path} FILES_MATCHING PATTERN "Makefile")

# Examples
file(GLOB matlab_examples "${CMAKE_SOURCE_DIR}/examples/matlab/*.m")
install(FILES ${matlab_examples} DESTINATION ${toolbox_install_path}/gtsam/examples)

# Tests
file(GLOB matlab_tests "${CMAKE_SOURCE_DIR}/tests/matlab/*.m")
install(FILES ${matlab_tests} DESTINATION ${toolbox_install_path}/gtsam/tests)

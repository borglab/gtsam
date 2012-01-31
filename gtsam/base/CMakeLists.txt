# Build convenience libraries
file(GLOB base_srcs "*.cpp")
add_library(base STATIC ${base_srcs})

# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam/base)

add_custom_target(check.base COMMAND ${CMAKE_CTEST_COMMAND})
add_custom_target(timing.base)

# Components to link tests in this subfolder against
set(base_local_libs
    CppUnitLite
    base
)

# Build tests
file(GLOB base_tests_srcs "tests/test*.cpp")
foreach(test_src ${base_tests_srcs})
    get_filename_component(test_base ${test_src} NAME_WE)
    set( test_bin base.${test_base} )
    message(STATUS "Adding Test ${test_bin}") 
    add_executable(${test_bin} EXCLUDE_FROM_ALL ${test_src})
    add_dependencies(check.base ${test_bin})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
    target_link_libraries(${test_bin} ${base_local_libs})
    add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
endforeach(test_src)

# Build timing scripts
file(GLOB base_timing_srcs "tests/time*.cpp") 
foreach(time_src ${base_timing_srcs})
    get_filename_component(time_base ${time_src} NAME_WE)
    set( time_bin base.${time_base} )
    message(STATUS "Adding Timing Benchmark ${time_bin}")
    add_executable(${time_bin} EXCLUDE_FROM_ALL ${time_src})
    add_dependencies(timing.base ${time_bin})
    target_link_libraries(${time_bin} ${base_local_libs})
    add_custom_target(${time_bin}.run ${EXECUTABLE_OUTPUT_PATH}${time_bin} ${ARGN})
endforeach(time_src)


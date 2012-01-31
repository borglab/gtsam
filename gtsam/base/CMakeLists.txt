# Build convenience libraries
file(GLOB base_srcs "*.cpp")
add_library(base STATIC ${base_srcs})

# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam/base)

add_custom_target(check.base)
add_custom_target(timing.base)

# Build tests
file(GLOB base_tests_srcs "tests/test*.cpp")
foreach(test_src ${base_tests_srcs})
    get_filename_component(test_base ${test_src} NAME_WE)
    set( test_bin base_${test_base} )
    add_executable(${test_bin} EXCLUDE_FROM_ALL ${test_src})
    add_dependencies(check.base ${test_bin})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
    target_link_libraries(${test_bin} CppUnitLite base)
    add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
endforeach(test_src)

# Build timing scripts
file(GLOB base_tests_srcs "tests/time*.cpp") 
foreach(time_src ${time_srcs})
    get_filename_component(time_base ${time_src} NAME_WE)
    set( time_bin ${time_base} )
    add_executable(${time_bin} EXCLUDE_FROM_ALL ${time_src})
    add_dependencies(timing.base ${time_bin})
    target_link_libraries(${time_bin} CppUnitLite base)
    add_custom_target(${time_base}.run ${EXECUTABLE_OUTPUT_PATH}${time_bin} ${ARGN})
endforeach(time_src)


# link back to base
add_dependencies(geometry base)

# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION include/gtsam/geometry)

add_custom_target(check.geometry COMMAND ${CMAKE_CTEST_COMMAND})
add_custom_target(timing.geometry)

# Components to link tests in this subfolder against
set(geometry_local_libs
   geometry
   base
   CppUnitLite
)

# Build tests
file(GLOB geometry_tests_srcs "tests/test*.cpp")
foreach(test_src ${geometry_tests_srcs})
    get_filename_component(test_base ${test_src} NAME_WE)
    set( test_bin geometry.${test_base} )
    message(STATUS "Adding Test ${test_bin}") 
    add_executable(${test_bin} ${test_src})
    add_dependencies(check.geometry ${test_bin})
    add_dependencies(check ${test_bin})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
    target_link_libraries(${test_bin} ${geometry_local_libs})
    add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
endforeach(test_src)

# Build timing scripts
file(GLOB geometry_timing_srcs "tests/time*.cpp") 
foreach(time_src ${geometry_timing_srcs})
    get_filename_component(time_base ${time_src} NAME_WE)
    set( time_bin geometry.${time_base} )
    message(STATUS "Adding Timing Benchmark ${time_bin}")
    add_executable(${time_bin} ${time_src})
    add_dependencies(timing.geometry ${time_bin})
    add_dependencies(timing ${time_bin})
    target_link_libraries(${time_bin} ${geometry_local_libs})
    add_custom_target(${time_bin}.run ${EXECUTABLE_OUTPUT_PATH}${time_bin} ${ARGN})
endforeach(time_src)


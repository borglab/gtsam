# link back to base
add_dependencies(slam nonlinear)

# Install headers
file(GLOB slam_headers "*.h")
install(FILES ${slam_headers} DESTINATION include/gtsam/slam)

add_custom_target(check.slam COMMAND ${CMAKE_CTEST_COMMAND})
add_custom_target(timing.slam)

# Components to link tests in this subfolder against
set(slam_local_libs
   slam
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
   CppUnitLite
)

# Build tests
file(GLOB slam_tests_srcs "tests/test*.cpp")
foreach(test_src ${slam_tests_srcs})
    get_filename_component(test_base ${test_src} NAME_WE)
    set( test_bin slam.${test_base} )
    message(STATUS "Adding Test ${test_bin}") 
    add_executable(${test_bin} ${test_src})
    add_dependencies(check.slam ${test_bin})
    add_dependencies(check ${test_bin})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
    target_link_libraries(${test_bin} ${slam_local_libs})
    add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
endforeach(test_src)

# Build timing scripts
file(GLOB slam_timing_srcs "tests/time*.cpp") 
foreach(time_src ${slam_timing_srcs})
    get_filename_component(time_base ${time_src} NAME_WE)
    set( time_bin slam.${time_base} )
    message(STATUS "Adding Timing Benchmark ${time_bin}")
    add_executable(${time_bin} ${time_src})
    add_dependencies(timing.slam ${time_bin})
    add_dependencies(timing ${time_bin})
    target_link_libraries(${time_bin} ${slam_local_libs})
    add_custom_target(${time_bin}.run ${EXECUTABLE_OUTPUT_PATH}${time_bin} ${ARGN})
endforeach(time_src)


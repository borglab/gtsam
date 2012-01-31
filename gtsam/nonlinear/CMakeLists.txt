# link back to base
add_dependencies(nonlinear linear)

# Install headers
file(GLOB nonlinear_headers "*.h")
install(FILES ${nonlinear_headers} DESTINATION include/gtsam/nonlinear)

# Components to link tests in this subfolder against
set(nonlinear_local_libs
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
   CppUnitLite
)

# Build tests
if (GTSAM_BUILD_TESTS)
    add_custom_target(check.nonlinear COMMAND ${CMAKE_CTEST_COMMAND})
    file(GLOB nonlinear_tests_srcs "tests/test*.cpp")
    foreach(test_src ${nonlinear_tests_srcs})
        get_filename_component(test_base ${test_src} NAME_WE)
        set( test_bin nonlinear.${test_base} )
        message(STATUS "Adding Test ${test_bin}") 
        add_executable(${test_bin} ${test_src})
        add_dependencies(check.nonlinear ${test_bin})
        add_dependencies(check ${test_bin})
        add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
        target_link_libraries(${test_bin} ${nonlinear_local_libs})
        add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
    endforeach(test_src)
endif (GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    add_custom_target(timing.nonlinear)
    file(GLOB nonlinear_timing_srcs "tests/time*.cpp") 
    foreach(time_src ${nonlinear_timing_srcs})
        get_filename_component(time_base ${time_src} NAME_WE)
        set( time_bin nonlinear.${time_base} )
        message(STATUS "Adding Timing Benchmark ${time_bin}")
        add_executable(${time_bin} ${time_src})
        add_dependencies(timing.nonlinear ${time_bin})
        add_dependencies(timing ${time_bin})
        target_link_libraries(${time_bin} ${nonlinear_local_libs})
        add_custom_target(${time_bin}.run ${EXECUTABLE_OUTPUT_PATH}${time_bin} ${ARGN})
    endforeach(time_src)
endif (GTSAM_BUILD_TIMING)


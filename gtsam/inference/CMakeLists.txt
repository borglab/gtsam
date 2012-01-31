# link back to previous convenience library
add_dependencies(inference base)

# Install headers
file(GLOB inference_headers "*.h")
install(FILES ${inference_headers} DESTINATION include/gtsam/inference)

# Components to link tests in this subfolder against
set(inference_local_libs
   inference
   geometry
   base
   ccolamd
   CppUnitLite
)

# Build tests
if(GTSAM_BUILD_TESTS)
    add_custom_target(check.inference COMMAND ${CMAKE_CTEST_COMMAND})
    file(GLOB inference_tests_srcs "tests/test*.cpp")
    foreach(test_src ${inference_tests_srcs})
        get_filename_component(test_base ${test_src} NAME_WE)
        set( test_bin inference.${test_base} )
        message(STATUS "Adding Test ${test_bin}") 
        add_executable(${test_bin} ${test_src})
        add_dependencies(check.inference ${test_bin})
        add_dependencies(check ${test_bin})
        add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
        target_link_libraries(${test_bin} ${inference_local_libs})
        add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
    endforeach(test_src)
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if(GTSAM_BUILD_TIMING)
    add_custom_target(timing.inference)
    file(GLOB inference_timing_srcs "tests/time*.cpp") 
    foreach(time_src ${inference_timing_srcs})
        get_filename_component(time_base ${time_src} NAME_WE)
        set( time_bin inference.${time_base} )
        message(STATUS "Adding Timing Benchmark ${time_bin}")
        add_executable(${time_bin} ${time_src})
        add_dependencies(timing.inference ${time_bin})
        add_dependencies(timing ${time_bin})
        target_link_libraries(${time_bin} ${inference_local_libs})
        add_custom_target(${time_bin}.run ${EXECUTABLE_OUTPUT_PATH}${time_bin} ${ARGN})
    endforeach(time_src)
endif(GTSAM_BUILD_TIMING)


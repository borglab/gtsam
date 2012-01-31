if (GTSAM_LINK_BINARIES_AGAINST_CONVENIENCE_LIBS)
    set(convenience_libs
       slam
       nonlinear
       linear
       inference
       geometry
       base
       ccolamd)
else (GTSAM_LINK_BINARIES_AGAINST_CONVENIENCE_LIBS)
    set(convenience_libs
       gtsam-static)
endif (GTSAM_LINK_BINARIES_AGAINST_CONVENIENCE_LIBS)

# Build tests
if (GTSAM_BUILD_TESTS)
    add_custom_target(check.tests COMMAND ${CMAKE_CTEST_COMMAND})
    file(GLOB tests_srcs "test*.cpp")
    foreach(test_src ${tests_srcs})
        get_filename_component(test_base ${test_src} NAME_WE)
        set( test_bin tests.${test_base} )
        message(STATUS "Adding Test ${test_bin}") 
        add_executable(${test_bin} ${test_src})
        add_dependencies(check.tests ${test_bin})
        add_dependencies(check ${test_bin})
        add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
        target_link_libraries(${test_bin} ${convenience_libs} CppUnitLite ${Boost_LIBRARIES})
        add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
    endforeach(test_src)
endif (GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    add_custom_target(timing.tests)
    file(GLOB timing_srcs "time*.cpp") 
    foreach(time_src ${timing_srcs})
        get_filename_component(time_base ${time_src} NAME_WE)
        set( time_bin tests.${time_base} )
        message(STATUS "Adding Timing Benchmark ${time_bin}")
        add_executable(${time_bin} ${time_src})
        add_dependencies(timing.tests ${time_bin})
        add_dependencies(timing ${time_bin})
        target_link_libraries(${time_bin} ${convenience_libs} CppUnitLite ${Boost_LIBRARIES})
        add_custom_target(${time_bin}.run ${EXECUTABLE_OUTPUT_PATH}${time_bin} ${ARGN})
    endforeach(time_src)
endif (GTSAM_BUILD_TIMING)

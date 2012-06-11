# Build a library of example domains, just for tests
file(GLOB test_lib_srcs "*.cpp")
file(GLOB test_srcs "test*.cpp")
file(GLOB time_srcs "time*.cpp")
list(REMOVE_ITEM test_lib_srcs ${test_srcs})
list(REMOVE_ITEM test_lib_srcs ${time_srcs})
add_library(test_lib STATIC ${test_lib_srcs})

# Assemble local libraries
set(tests_local_libs
   test_lib
   slam
   nonlinear
   linear
   discrete
   inference
   geometry
   base
   ccolamd
   CppUnitLite)

# exclude certain files
# note the source dir on each 
set (tests_exclude
    #"${CMAKE_CURRENT_SOURCE_DIR}/testOccupancyGrid.cpp"
)

if(MSVC)
  add_definitions("/bigobj") # testSerializationSLAM needs this
endif()

# Build tests
if (GTSAM_BUILD_TESTS)
    # Subdirectory target for tests
    add_custom_target(check.tests COMMAND ${CMAKE_CTEST_COMMAND})
    set(is_test TRUE)

    # Build grouped tests
    gtsam_add_grouped_scripts("tests"               # Use subdirectory as group label
    "test*.cpp" check "Test"                      # Standard for all tests
    "${tests_local_libs}" "gtsam-static;CppUnitLite;test_lib" "${tests_exclude}"  # Pass in linking and exclusion lists
    ${is_test})                                         # Set all as tests
endif (GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    # Subdirectory target for timing - does not actually execute the scripts
    add_custom_target(timing.tests)
    set(is_test FALSE)

    # Build grouped benchmarks
    gtsam_add_grouped_scripts("tests"               # Use subdirectory as group label
    "time*.cpp" timing "Timing Benchmark"         # Standard for all timing scripts
    "${tests_local_libs}" "gtsam-static;CppUnitLite;test_lib" "${tests_exclude}"   # Pass in linking and exclusion lists
    ${is_test})
endif (GTSAM_BUILD_TIMING)

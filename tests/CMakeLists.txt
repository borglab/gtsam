# exclude certain files
# note the source dir on each 
set (tests_exclude "")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang") # might not be best test - Richard & Jason & Frank
	# clang linker segfaults on large testSerializationSLAM 
	list (APPEND tests_exclude "testSerializationSLAM.cpp") 
endif()

# Build tests
gtsamAddTestsGlob(tests "test*.cpp" "${tests_exclude}" "gtsam")

if(MSVC)
	set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationSLAM.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    # Subdirectory target for timing - does not actually execute the scripts
    add_custom_target(timing.tests)
    set(is_test FALSE)

    # Build grouped benchmarks
    gtsam_add_grouped_scripts("tests"               # Use subdirectory as group label
    "time*.cpp" timing "Timing Benchmark"         # Standard for all timing scripts
    "${tests_full_libs}" "${tests_full_libs}" "${tests_exclude}"   # Pass in linking and exclusion lists
    ${is_test})
endif (GTSAM_BUILD_TIMING)

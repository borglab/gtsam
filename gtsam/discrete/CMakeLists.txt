# Install headers
set(subdir discrete)
file(GLOB discrete_headers "*.h")
# FIXME: exclude headers
install(FILES ${discrete_headers} DESTINATION include/gtsam2/discrete)

# Set up library dependencies
set (discrete_local_libs 
    discrete)

set (discrete_full_libs
    gtsam2-static)

# Exclude tests that don't work
set (discrete_excluded_tests
"${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactorGraph.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/tests/testPotentialTable.cpp")

# Add all tests
gtsam_add_subdir_tests(discrete "${discrete_local_libs}" "${discrete_full_libs}" "${discrete_excluded_tests}")

# add examples
foreach(example schedulingExample schedulingQuals12)
	add_executable(${example} "examples/${example}.cpp")
	add_dependencies(${example} gtsam2-static)
	target_link_libraries(${example} ${Boost_LIBRARIES} gtsam2-static)
	add_custom_target(${example}.run ${EXECUTABLE_OUTPUT_PATH}${example} ${ARGN})
endforeach(example)

# Build timing scripts
#if (GTSAM_BUILD_TIMING)
#    gtsam_add_timing(discrete "${discrete_local_libs}")
#endif(GTSAM_BUILD_TIMING)


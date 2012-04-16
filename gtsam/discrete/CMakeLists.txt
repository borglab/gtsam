# Install headers
set(subdir discrete)
file(GLOB discrete_headers "*.h")
# FIXME: exclude headers
install(FILES ${discrete_headers} DESTINATION include/gtsam/discrete)

# Set up library dependencies
set (discrete_local_libs 
   discrete
   inference
   base
   ccolamd
)

# Exclude tests that don't work
set (discrete_excluded_tests
"${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactorGraph.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/tests/testPotentialTable.cpp")

# Add all tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(discrete "${discrete_local_libs}" "gtsam-static" "${discrete_excluded_tests}")
endif()

# add examples
foreach(example schedulingExample schedulingQuals12)
	add_executable(${example} "examples/${example}.cpp")
	add_dependencies(${example} gtsam-static)
	target_link_libraries(${example} gtsam-static)
	add_custom_target(${example}.run ${EXECUTABLE_OUTPUT_PATH}${example} ${ARGN})
endforeach(example)

# Build timing scripts
#if (GTSAM_BUILD_TIMING)
#    gtsam_add_timing(discrete "${discrete_local_libs}")
#endif(GTSAM_BUILD_TIMING)


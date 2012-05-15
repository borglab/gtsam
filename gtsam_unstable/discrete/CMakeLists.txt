# Install headers
file(GLOB discrete_headers "*.h")
install(FILES ${discrete_headers} DESTINATION include/gtsam_unstable/discrete)

# Components to link tests in this subfolder against
set(discrete_local_libs 
   discrete_unstable
   discrete
   inference
   base
   ccolamd
)

set (discrete_full_libs
    gtsam-static
    gtsam_unstable-static)

# Exclude tests that don't work
set (discrete_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(discrete_unstable "${discrete_local_libs}" "${discrete_full_libs}" "${discrete_excluded_tests}") 

# add examples
foreach(example schedulingExample schedulingQuals12)
	add_executable(${example} "examples/${example}.cpp")
	add_dependencies(${example} gtsam-static)
	target_link_libraries(${example} gtsam-static)
	add_custom_target(${example}.run ${EXECUTABLE_OUTPUT_PATH}${example} ${ARGN})
endforeach(example)


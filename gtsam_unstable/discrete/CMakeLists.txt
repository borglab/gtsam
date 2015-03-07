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
#set (discrete_excluded_tests 
#"${CMAKE_CURRENT_SOURCE_DIR}/tests/testScheduler.cpp"
#)


# Add all tests
gtsam_add_subdir_tests(discrete_unstable "${discrete_local_libs}" "${discrete_full_libs}" "${discrete_excluded_tests}") 
add_dependencies(check.unstable check.discrete_unstable)

# List examples to build - comment out here to exclude from compilation
set(discrete_unstable_examples
schedulingExample
schedulingQuals12
)

if (GTSAM_BUILD_EXAMPLES)
    foreach(example ${discrete_unstable_examples})
    	add_executable(${example} "examples/${example}.cpp")
    	
    	# Disable building during make all/install
        if (GTSAM_ENABLE_INSTALL_EXAMPLE_FIX)
            set_target_properties(${example} PROPERTIES EXCLUDE_FROM_ALL ON)
        endif()
    	
		if(NOT MSVC)
			add_dependencies(examples ${example})
			add_custom_target(${example}.run ${EXECUTABLE_OUTPUT_PATH}${example} ${ARGN})
		endif()
    	
    	add_dependencies(${example} gtsam-static gtsam_unstable-static)
    	target_link_libraries(${example} gtsam-static gtsam_unstable-static)
    endforeach(example)
endif (GTSAM_BUILD_EXAMPLES)

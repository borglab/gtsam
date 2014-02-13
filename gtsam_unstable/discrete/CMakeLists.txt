# Install headers
file(GLOB discrete_headers "*.h")
install(FILES ${discrete_headers} DESTINATION include/gtsam_unstable/discrete)

# Add all tests
add_subdirectory(tests)

# List examples to build - comment out here to exclude from compilation
set(discrete_unstable_examples
schedulingExample
schedulingQuals12
schedulingQuals13
)

if (GTSAM_BUILD_EXAMPLES)
    foreach(example ${discrete_unstable_examples})
    	add_executable(${example} "examples/${example}.cpp")
    	
    	# Disable building during make all/install
        if (GTSAM_ENABLE_INSTALL_EXAMPLE_FIX)
            set_target_properties(${example} PROPERTIES EXCLUDE_FROM_ALL ON)
        endif()
    	
		if(NOT MSVC AND NOT XCODE_VERSION)
			add_dependencies(examples ${example})
			add_custom_target(${example}.run ${EXECUTABLE_OUTPUT_PATH}${example} ${ARGN})
		endif()
    	
    	target_link_libraries(${example} gtsam gtsam_unstable)
    endforeach(example)
endif (GTSAM_BUILD_EXAMPLES)

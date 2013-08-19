# Install headers
file(GLOB linear_headers "*.h")
install(FILES ${linear_headers} DESTINATION include/gtsam/linear)

# Files to exclude from compilation of tests and timing scripts
set(linear_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test
#    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(linear "${gtsam-default}" "${gtsam-default}" "${linear_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

if(MSVC)
	set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/tests/testSerializationLinear.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(linear "${gtsam-default}" "${gtsam-default}" "${linear_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

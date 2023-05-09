# exclude certain files
# note the source dir on each 
set (excluded_tests "")

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") # might not be best test - Richard & Jason & Frank
	# clang linker segfaults on large testSerializationSlam 
	list (APPEND excluded_tests "testSerializationSlam.cpp") 
endif()

if (NOT GTSAM_USE_BOOST_FEATURES)
	list(APPEND excluded_tests "testGncOptimizer.cpp")
	list(APPEND excluded_tests "testGraph.cpp")
endif()

if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
	list(APPEND excluded_tests "testSerializationSLAM.cpp")
endif()

# Build tests
gtsamAddTestsGlob(tests "test*.cpp" "${excluded_tests}" "gtsam")

if(MSVC)
	set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationSlam.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

# exclude certain files
# note the source dir on each 
set (tests_exclude "")

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") # might not be best test - Richard & Jason & Frank
	# clang linker segfaults on large testSerializationSlam 
	list (APPEND tests_exclude "testSerializationSlam.cpp") 
endif()

if (NOT GTSAM_USE_BOOST_FEATURES)
	list(APPEND tests_exclude "testGncOptimizer.cpp")
endif()

if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
	list(APPEND tests_exclude "testSerializationSLAM.cpp")
endif()

# Build tests
gtsamAddTestsGlob(tests "test*.cpp" "${tests_exclude}" "gtsam")

if(MSVC)
	set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationSlam.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

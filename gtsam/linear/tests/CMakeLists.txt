# if GTSAM_ENABLE_BOOST_SERIALIZATION is OFF then exclude some tests
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
  # create a semicolon seperated list of files to exclude
	set(EXCLUDE_TESTS "testSerializationLinear.cpp")
else()
  set(EXCLUDE_TESTS "")
endif()

if(NOT GTSAM_ENABLE_CUDA AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaLinearSolver.cpp")
  list(APPEND EXCLUDE_TESTS "testCudaLinearSolver.cpp")
endif()

gtsamAddTestsGlob(linear "test*.cpp" "${EXCLUDE_TESTS}" "gtsam")

# Exercise default callbacks from a separately linked consumer whose inline
# functions are not coalesced with the library's copies (as with Windows DLLs).
if(NOT MSVC AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testGaussianFactorGraph.cpp"
    APPEND PROPERTY COMPILE_OPTIONS "-fvisibility-inlines-hidden")
endif()

# Set properties to serialization target if Boost serialization is enabled and MSVC
if (GTSAM_ENABLE_BOOST_SERIALIZATION AND MSVC)
		set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationLinear.cpp"
			APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

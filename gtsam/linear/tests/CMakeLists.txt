# if GTSAM_ENABLE_BOOST_SERIALIZATION is OFF then exclude some tests
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
  # create a semicolon seperated list of files to exclude
	set(EXCLUDE_TESTS "testSerializationLinear.cpp")
else()
  set(EXCLUDE_TESTS "")
endif()

gtsamAddTestsGlob(linear "test*.cpp" "${EXCLUDE_TESTS}" "gtsam")

# Set properties to serialization target if Boost serialization is enabled and MSVC
if (GTSAM_ENABLE_BOOST_SERIALIZATION AND MSVC)
		set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationLinear.cpp"
			APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

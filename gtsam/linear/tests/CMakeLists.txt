gtsamAddTestsGlob(linear "test*.cpp" "" "gtsam")

if(MSVC)
	set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationLinear.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

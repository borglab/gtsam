# exclude certain files
# note the source dir on each 
set (tests_exclude "")

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") # might not be best test - Richard & Jason & Frank
	# clang linker segfaults on large testSerializationSlam 
	list (APPEND tests_exclude "testSerializationSlam.cpp") 
endif()

include(FetchContent)

FetchContent_Declare( tracy GIT_REPOSITORY https://github.com/wolfpld/tracy.git GIT_TAG master GIT_SHALLOW TRUE GIT_PROGRESS TRUE )

option(TRACY_NO_EXIT ON)

FetchContent_MakeAvailable(tracy)

# Build tests
gtsamAddTestsGlob(tests "test*.cpp" "${tests_exclude}" "gtsam")

target_link_libraries(testTBBFail PUBLIC TracyClient)

if(MSVC)
	set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationSlam.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

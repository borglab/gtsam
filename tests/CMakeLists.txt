# exclude certain files
# note the source dir on each 
set (excluded_tests "")

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") # might not be best test - Richard & Jason & Frank
	# clang linker segfaults on large testSerializationSlam 
	list (APPEND excluded_tests "testSerializationSlam.cpp") 
endif()

if (NOT GTSAM_USE_BOOST_FEATURES)
	list(APPEND excluded_tests "testGraph.cpp")
endif()

if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION OR GTSAM_SINGLE_TEST_EXE)
	list(APPEND excluded_tests "testSerializationSlam.cpp")
endif()

# Build the Ceres-dependent test separately when system Ceres is available.
list(APPEND excluded_tests "testAdaptAutoDiff.cpp")

if(NOT GTSAM_ALLOW_DEPRECATED_SINCE_V43)
  list(APPEND excluded_tests "testHeaderMoveCompatibility.cpp")
endif()

# Build tests
gtsamAddTestsGlob(tests "test*.cpp" "${excluded_tests}" "gtsam")

if(GTSAM_BUILD_TESTS)
  if(Ceres_FOUND)
    message(STATUS "Found Ceres ${CERES_VERSION}; enabling testAdaptAutoDiff")
    gtsamAddTestsGlob(ceres_autodiff "testAdaptAutoDiff.cpp" ""
                      "gtsam;Ceres::ceres")
  else()
    message(STATUS "Ceres not found; testAdaptAutoDiff disabled")
  endif()
endif()

# Exercise Values RTTI across a hidden-by-default shared-library boundary.
if(GTSAM_BUILD_TESTS AND GTSAM_SHARED_LIB)
  add_library(testValuesCrossLibraryHelper SHARED
              values_cross_library/ValuesCrossLibrary.cpp)
  target_compile_definitions(testValuesCrossLibraryHelper
                             PRIVATE VALUES_CROSS_LIBRARY_EXPORTS)
  target_include_directories(testValuesCrossLibraryHelper
                             PUBLIC values_cross_library)
  target_link_libraries(testValuesCrossLibraryHelper PRIVATE gtsam)
  set_target_properties(testValuesCrossLibraryHelper PROPERTIES
                        CXX_VISIBILITY_PRESET hidden
                        VISIBILITY_INLINES_HIDDEN YES
                        EXCLUDE_FROM_ALL ON)
  gtsam_apply_build_flags(testValuesCrossLibraryHelper)

  add_executable(testValuesCrossLibrary
                 values_cross_library/testValuesCrossLibrary.cpp)
  target_link_libraries(testValuesCrossLibrary PRIVATE CppUnitLite gtsam
                                                     testValuesCrossLibraryHelper)
  set_target_properties(testValuesCrossLibrary PROPERTIES
                        CXX_VISIBILITY_PRESET hidden
                        VISIBILITY_INLINES_HIDDEN YES
                        EXCLUDE_FROM_ALL ON)
  gtsam_apply_build_flags(testValuesCrossLibrary)

  add_test(NAME testValuesCrossLibrary COMMAND testValuesCrossLibrary)
  add_dependencies(check.tests testValuesCrossLibrary)
  add_dependencies(check testValuesCrossLibrary)
  add_dependencies(all.tests testValuesCrossLibrary)

  if(NOT MSVC AND NOT XCODE_VERSION)
    add_custom_target(testValuesCrossLibrary.run
                      COMMAND $<TARGET_FILE:testValuesCrossLibrary>
                      DEPENDS testValuesCrossLibrary)
  endif()
endif()

if(MSVC)
	set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/testSerializationSlam.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

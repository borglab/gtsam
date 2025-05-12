set(test_link_libraries gtsam)
set(tests_excluded "")

# Decide whether to use installed GeographicLib or the one built within GTSAM.
# This logic should mirror the check in gtsam/3rdparty/CMakeLists.txt

# Check if we are building the bundled GeographicLib
if(GTSAM_INSTALL_GEOGRAPHICLIB)
  # If building the bundled version, include its source headers and link its main target.
  # Linking against the 'GeographicLib' target is preferred as it resolves correctly
  # regardless of whether GeographicLib was built as static or shared internally.
  # This avoids hardcoding '_STATIC' or '_SHARED'.
  message(STATUS "Navigation tests: Linking bundled GeographicLib target.")
  include_directories(${PROJECT_SOURCE_DIR}/gtsam/3rdparty/GeographicLib/include)
  list(APPEND test_link_libraries GeographicLib) # Use the main target name

# Check if a system GeographicLib was found (consistent with 3rdparty check)
elseif(GeographicLib_FOUND)
  # If not building the bundled one, but a system one was found, use it.
  message(STATUS "Navigation tests: Linking system GeographicLib target.")
  include_directories(${GeographicLib_INCLUDE_DIRS})
  list(APPEND test_link_libraries ${GeographicLib_LIBRARIES}) # Use variables set by FindGeographicLib

else()
  # If not building bundled and no system library found, exclude tests requiring it.
  message(STATUS "Navigation tests: GeographicLib not found, excluding related tests.")
  set(tests_excluded testGeographicLib.cpp testGPSFactor.cpp testMagFactor.cpp)
endif()

# if GTSAM_ENABLE_BOOST_SERIALIZATION is OFF then exclude some tests
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
  list(APPEND tests_excluded testSerializationNavigation.cpp)
endif()

gtsamAddTestsGlob(navigation "test*.cpp" "${tests_excluded}" "${test_link_libraries}")

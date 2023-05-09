set(test_link_libraries gtsam)
set(tests_excluded "")

# Decide whether to use installed GeographicLib or the one built in GTSAM.
# If we are not installing GeographicLib and it's not installed already,
# disable the unit tests that require it.
if(GTSAM_INSTALL_GEOGRAPHICLIB)
  # If we're installing GeographicLib, use the one we're compiling
  include_directories(${PROJECT_SOURCE_DIR}/gtsam/3rdparty/GeographicLib/include)
  if(MSVC)
    list(APPEND test_link_libraries GeographicLib_STATIC)
  else()
    list(APPEND test_link_libraries GeographicLib)
  endif()

else()
  if(GeographicLib_LIBRARIES)
    # If we're not installing, but it's already installed, use the installed one
    include_directories(${GeographicLib_INCLUDE_DIRS})
    list(APPEND test_link_libraries ${GeographicLib_LIBRARIES})
  else()
    # We don't have GeographicLib
    set(tests_excluded testGeographicLib.cpp testGPSFactor.cpp testMagFactor.cpp)
  endif()
endif()

# if GTSAM_ENABLE_BOOST_SERIALIZATION is OFF then exclude some tests
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
  list(APPEND tests_excluded testSerializationNavigation.cpp)
endif()

gtsamAddTestsGlob(navigation "test*.cpp" "${tests_excluded}" "${test_link_libraries}")

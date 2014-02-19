# This file installs the scripts from this directory that may be used in other 
# projects.  See README.txt in this directory for documentation.

# Set the install directory depending on the platform so it will be found by
# find_package(GTSAM-CMakeTools)
if(WIN32 AND NOT CYGWIN)
	set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/CMake")
else()
	set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake")
endif()

# Install scripts
install(FILES
  GTSAM-CMakeToolsConfig.cmake
  Config.cmake.in
  dllexport.h.in
  GtsamBuildTypes.cmake
  GtsamMakeConfigFile.cmake
  GtsamMatlabWrap.cmake
  GtsamPrinting.cmake
  GtsamPythonWrap.cmake
  GtsamTesting.cmake
  GtsamTestingObsolete.cmake
  DESTINATION "${SCRIPT_INSTALL_DIR}/GTSAM-CMakeTools")



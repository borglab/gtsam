# This file installs the scripts from this directory that may be used in other 
# projects.  See README.txt in this directory for documentation.

# Set the install directory depending on the platform so it will be found by
# find_package(GTSAMCMakeTools)
if(WIN32 AND NOT CYGWIN)
	set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/CMake")
else()
	set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake")
endif()

# Install scripts
install(FILES
  GTSAMCMakeToolsConfig.cmake
  Config.cmake.in
  dllexport.h.in
  GtsamBuildTypes.cmake
  GtsamMakeConfigFile.cmake
  GtsamTesting.cmake
  GtsamPrinting.cmake
  FindNumPy.cmake
  README.html
  DESTINATION "${SCRIPT_INSTALL_DIR}/GTSAMCMakeTools")

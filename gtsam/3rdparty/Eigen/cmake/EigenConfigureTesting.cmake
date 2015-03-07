include(EigenTesting)
include(CheckCXXSourceCompiles)

# configure the "site" and "buildname" 
ei_set_sitename()

# retrieve and store the build string
ei_set_build_string()

add_custom_target(buildtests)
add_custom_target(check COMMAND "ctest")
add_dependencies(check buildtests)

# check whether /bin/bash exists
find_file(EIGEN_BIN_BASH_EXISTS "/bin/bash" PATHS "/" NO_DEFAULT_PATH)

# CMake/Ctest does not allow us to change the build command,
# so we have to workaround by directly editing the generated DartConfiguration.tcl file
# save CMAKE_MAKE_PROGRAM
set(CMAKE_MAKE_PROGRAM_SAVE ${CMAKE_MAKE_PROGRAM})
# and set a fake one
set(CMAKE_MAKE_PROGRAM "@EIGEN_MAKECOMMAND_PLACEHOLDER@")

# This call activates testing and generates the DartConfiguration.tcl
include(CTest)

# overwrite default DartConfiguration.tcl
# The worarounds are different for each version of the MSVC IDE
if(MSVC_IDE)
  if(MSVC_VERSION EQUAL 1600) # MSVC 2010
    set(EIGEN_MAKECOMMAND_PLACEHOLDER "${CMAKE_MAKE_PROGRAM_SAVE} buildtests.vcxproj /p:Configuration=\${CTEST_CONFIGURATION_TYPE} \n# ")
  else() # MSVC 2008 (TODO check MSVC 2005)
    set(EIGEN_MAKECOMMAND_PLACEHOLDER "${CMAKE_MAKE_PROGRAM_SAVE} Eigen.sln /build \"Release\" /project buildtests \n# ")
  endif()
else()
  # for make and nmake
  set(EIGEN_MAKECOMMAND_PLACEHOLDER "${CMAKE_MAKE_PROGRAM_SAVE} buildtests")
endif()

# copy ctest properties, which currently
# o raise the warning levels
configure_file(${CMAKE_BINARY_DIR}/DartConfiguration.tcl ${CMAKE_BINARY_DIR}/DartConfiguration.tcl)

# restore default CMAKE_MAKE_PROGRAM
set(CMAKE_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM_SAVE})
# un-set temporary variables so that it is like they never existed. 
# CMake 2.6.3 introduces the more logical unset() syntax for this.
set(CMAKE_MAKE_PROGRAM_SAVE) 
set(EIGEN_MAKECOMMAND_PLACEHOLDER)

configure_file(${CMAKE_SOURCE_DIR}/CTestCustom.cmake.in ${CMAKE_BINARY_DIR}/CTestCustom.cmake)

# some documentation of this function would be nice
ei_init_testing()

# configure Eigen related testing options
option(EIGEN_NO_ASSERTION_CHECKING "Disable checking of assertions using exceptions" OFF)
option(EIGEN_DEBUG_ASSERTS "Enable advanced debuging of assertions" OFF)

if(CMAKE_COMPILER_IS_GNUCXX)
  option(EIGEN_COVERAGE_TESTING "Enable/disable gcov" OFF)
  if(EIGEN_COVERAGE_TESTING)
    set(COVERAGE_FLAGS "-fprofile-arcs -ftest-coverage")
    set(CTEST_CUSTOM_COVERAGE_EXCLUDE "/test/")
  else(EIGEN_COVERAGE_TESTING)
    set(COVERAGE_FLAGS "")
  endif(EIGEN_COVERAGE_TESTING)
  if(EIGEN_TEST_C++0x)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
  endif(EIGEN_TEST_C++0x)
  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_FLAGS} -g2")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${COVERAGE_FLAGS} -O2 -g2")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${COVERAGE_FLAGS} -fno-inline-functions")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${COVERAGE_FLAGS} -O0 -g3")
  endif(CMAKE_SYSTEM_NAME MATCHES Linux)
elseif(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_WARNINGS /D_SCL_SECURE_NO_WARNINGS")
endif(CMAKE_COMPILER_IS_GNUCXX)

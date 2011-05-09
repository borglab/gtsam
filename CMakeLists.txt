CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT (gtsam)
if (NOT CMAKE_INSTALL_PREFIX)
	set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE INTERNAL "" FORCE)
endif()

set(CMAKE_CXX_FLAGS "-O2 -g -Wall -march=core2 -msse2 -DNDEBUG -DGT_USE_CBLAS -DGT_USE_LAPACK")

INCLUDE_DIRECTORIES ("${CMAKE_SOURCE_DIR}")

set(BOOST_ROOT ${BOOST_ROOT} CACHE PATH "The boost root path")
INCLUDE_DIRECTORIES ("${BOOST_ROOT}")
LINK_DIRECTORIES ("${BOOST_ROOT}/stage/lib")

set(CCOLAMD_ROOT ${CCOLAMD_ROOT} CACHE PATH "The ccolamd root path")
INCLUDE_DIRECTORIES ("${CCOLAMD_ROOT}")
LINK_DIRECTORIES ("${CCOLAMD_ROOT}")
set(libs_support ${libs_support} ccolamd)


set(USE_GOTOBLAS2 ${USE_GOTOBLAS2} CACHE BOOL "USE GotoBlas2 or not")
if (USE_GOTOBLAS2)
	set(GOTOBLAS2_ROOT ${GOTOBLAS2_ROOT} CACHE PATH "The gotoblas2 root path")
	INCLUDE_DIRECTORIES ("${GOTOBLAS2_ROOT}")
	set(libs_support ${libs_support} goto2)
else ()
	set(libs_support ${libs_support} cblas lapack atlas)	
endif ()

set(BUILD_EXAMPLES TRUE CACHE BOOL "Turn on the flag to build examples")
set(BUILD_TESTS TRUE CACHE BOOL "Turn on the flag to build tests")
set(BUILD_TIMINGS TRUE CACHE BOOL "Turn on the flag to build timings")

enable_testing()
add_subdirectory(CppUnitLite)
add_subdirectory(gtsam)

set(libs_gtsam base geometry inference linear nonlinear slam)

add_subdirectory(examples)
add_subdirectory(tests)





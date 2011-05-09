CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

set (lib_target "slam")
set (slam_src simulated2D.cpp smallExample.cpp simulated2DOriented.cpp Simulated3D.cpp pose2SLAM.cpp dataset.cpp planarSLAM.cpp pose3SLAM.cpp visualSLAM.cpp)

############################# COMPILE LIBRARY
add_library (${lib_target} STATIC ${slam_src})
target_link_libraries(${lib_target} base geometry inference linear nonlinear ccolamd)

# TESTS
set(libs_support ${libs_support} CppUnitLite)

if (BUILD_TESTS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};testGeneralSFMFactor_Cal3Bundler")
	set(PROGRAMS "${PROGRAMS};testGeneralSFMFactor")
	set(PROGRAMS "${PROGRAMS};testPlanarSLAM")
	set(PROGRAMS "${PROGRAMS};testPose2Factor")
	set(PROGRAMS "${PROGRAMS};testPose2Prior")
	set(PROGRAMS "${PROGRAMS};testPose2SLAM")
	set(PROGRAMS "${PROGRAMS};testPose2Values")
	set(PROGRAMS "${PROGRAMS};testPose3Factor")
	set(PROGRAMS "${PROGRAMS};testPose3SLAM")
	set(PROGRAMS "${PROGRAMS};testPose3Values")
	set(PROGRAMS "${PROGRAMS};testSimulated2D")
	set(PROGRAMS "${PROGRAMS};testSimulated2DOriented")
	set(PROGRAMS "${PROGRAMS};testSimulated3D")
	set(PROGRAMS "${PROGRAMS};testStereoFactor")
	set(PROGRAMS "${PROGRAMS};testVSLAMFactor")
	set(PROGRAMS "${PROGRAMS};testVSLAMGraph")
	set(PROGRAMS "${PROGRAMS};testVSLAMValues")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
		add_test(${program} ${program})
	endforeach(program)
endif ()

if (BUILD_TIMINGS)
	set(PROGRAMS "")
#	set(PROGRAMS "${PROGRAMS};timeSymbolMaps")
endif ()



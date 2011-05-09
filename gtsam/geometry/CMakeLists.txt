CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

set (lib_target "geometry")
file (GLOB geometry_src "./*.cpp")

############################# COMPILE LIBRARY

add_library (${lib_target} STATIC ${geometry_src})
target_link_libraries (${lib_target} base)

# TESTS

set(libs_support ${libs_support} CppUnitLite)

if (BUILD_TESTS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};testCal3Bundler")
	set(PROGRAMS "${PROGRAMS};testCal3DS2")
	set(PROGRAMS "${PROGRAMS};testCal3_S2")
	set(PROGRAMS "${PROGRAMS};testCalibratedCamera")
	set(PROGRAMS "${PROGRAMS};testFundamental")
	set(PROGRAMS "${PROGRAMS};testHomography2")
	set(PROGRAMS "${PROGRAMS};testPoint2")
	set(PROGRAMS "${PROGRAMS};testPoint3")
	set(PROGRAMS "${PROGRAMS};testRot2")
	set(PROGRAMS "${PROGRAMS};testRot3")
	set(PROGRAMS "${PROGRAMS};testSimpleCamera")
	set(PROGRAMS "${PROGRAMS};testStereoCamera")
	set(PROGRAMS "${PROGRAMS};testTensors")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
		add_test(${program} ${program})
	endforeach(program)
endif ()

if (BUILD_TIMINGS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};timeCalibratedCamera")
	set(PROGRAMS "${PROGRAMS};timeRot3")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
	endforeach(program)
endif ()



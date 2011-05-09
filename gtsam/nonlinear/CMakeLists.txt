CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

set (lib_target "nonlinear")
file (GLOB nonlinear_src "./*.cpp")

############################# COMPILE LIBRARY
add_library (${lib_target} STATIC ${nonlinear_src})

# TESTS
set(libs_support ${libs_support} CppUnitLite base)

if (BUILD_TESTS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};testKey")
	set(PROGRAMS "${PROGRAMS};testLieValues")
	set(PROGRAMS "${PROGRAMS};testOrdering")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
		add_test(${program} ${program})
	endforeach(program)
endif ()

if (BUILD_TIMINGS)
	set(PROGRAMS "")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
	endforeach(program)
endif ()



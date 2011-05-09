CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

set (lib_target "inference")
file (GLOB inference_src "./*.cpp")

############################# COMPILE LIBRARY
add_library (${lib_target} STATIC ${inference_src})
target_link_libraries (${lib_target} base geometry ccolamd)


# TESTS
set(libs_support ${libs_support} CppUnitLite)


if (BUILD_TESTS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};testBayesTree")
#	set(PROGRAMS "${PROGRAMS};testBinaryBayesNet")
	set(PROGRAMS "${PROGRAMS};testClusterTree")
	set(PROGRAMS "${PROGRAMS};testConditional")
	set(PROGRAMS "${PROGRAMS};testEliminationTree")
	set(PROGRAMS "${PROGRAMS};testFactorGraph")
	set(PROGRAMS "${PROGRAMS};testInference")
	set(PROGRAMS "${PROGRAMS};testISAM")
	set(PROGRAMS "${PROGRAMS};testJunctionTree")
	set(PROGRAMS "${PROGRAMS};testSymbolicBayesNet")
	set(PROGRAMS "${PROGRAMS};testSymbolicFactor")
	set(PROGRAMS "${PROGRAMS};testSymbolicFactorGraph")
	set(PROGRAMS "${PROGRAMS};testVariableIndex")
	set(PROGRAMS "${PROGRAMS};testVariableSlots")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
		add_test(${program} ${program})
	endforeach(program)
endif ()

if (BUILD_TIMINGS)
	set(PROGRAMS "")
#	set(PROGRAMS "${PROGRAMS};timeSymbolMaps")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
	endforeach(program)
endif ()




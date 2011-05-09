CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# TESTS

set(libs_support ${libs_support} CppUnitLite ${libs_gtsam})

if (BUILD_TESTS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};testGaussianBayesNet")
	set(PROGRAMS "${PROGRAMS};testGaussianFactor")
	set(PROGRAMS "${PROGRAMS};testGaussianFactorGraph")
	set(PROGRAMS "${PROGRAMS};testGaussianISAM")
	set(PROGRAMS "${PROGRAMS};testGraph")
	set(PROGRAMS "${PROGRAMS};testInference")
	set(PROGRAMS "${PROGRAMS};testGaussianJunctionTree")
	set(PROGRAMS "${PROGRAMS};testNonlinearEquality")
	set(PROGRAMS "${PROGRAMS};testNonlinearFactor")
	set(PROGRAMS "${PROGRAMS};testNonlinearFactorGraph")
	set(PROGRAMS "${PROGRAMS};testNonlinearOptimizer")
	set(PROGRAMS "${PROGRAMS};testSymbolicBayesNet")
	set(PROGRAMS "${PROGRAMS};testSymbolicFactorGraph")
	set(PROGRAMS "${PROGRAMS};testTupleValues")
	set(PROGRAMS "${PROGRAMS};testNonlinearISAM")
	foreach(program ${PROGRAMS})
		set (program_exe "${program}-global")
		add_executable(${program_exe} "${program}.cpp")
		target_link_libraries(${program_exe} ${lib_target} ${libs_support})
		add_test(${program_exe} ${program_exe})
	endforeach(program)
endif ()


#check_PROGRAMS += testSerialization

if (BUILD_TIMINGS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};timeGaussianFactorGraph")
	set(PROGRAMS "${PROGRAMS};timeSequentialOnDataset")
	set(PROGRAMS "${PROGRAMS};timeMultifrontalOnDataset")
	foreach(program ${PROGRAMS})
		set (program_exe "${program}-global")
		add_executable(${program_exe} "${program}.cpp")
		target_link_libraries(${program_exe} ${lib_target} ${libs_support})
	endforeach(program)
endif ()


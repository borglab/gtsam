CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

set (lib_target "linear")
#file (GLOB linear_src "./*.cpp")
set (linear_src NoiseModel.cpp Errors.cpp Sampler.cpp GaussianSequentialSolver.cpp GaussianMultifrontalSolver.cpp 
				JacobianFactor.cpp HessianFactor.cpp GaussianFactor.cpp GaussianFactorGraph.cpp 
				GaussianJunctionTree.cpp  GaussianConditional.cpp GaussianBayesNet.cpp GaussianISAM.cpp)

############################# COMPILE LIBRARY
add_library (${lib_target} STATIC ${linear_src})
target_link_libraries(${lib_target} base inference ccolamd)

# TESTS
set(libs_support ${libs_support} CppUnitLite)

if (BUILD_TESTS)
	set(PROGRAMS "")
#	set(PROGRAMS "${PROGRAMS};testBayesNetPreconditioner")
	set(PROGRAMS "${PROGRAMS};testErrors")
	set(PROGRAMS "${PROGRAMS};testGaussianConditional")
	set(PROGRAMS "${PROGRAMS};testGaussianFactor")
	set(PROGRAMS "${PROGRAMS};testGaussianFactorGraph")
	set(PROGRAMS "${PROGRAMS};testGaussianJunctionTree")
	set(PROGRAMS "${PROGRAMS};testHessianFactor")
	set(PROGRAMS "${PROGRAMS};testNoiseModel")
	set(PROGRAMS "${PROGRAMS};testSampler")
#	set(PROGRAMS "${PROGRAMS};testVectorBTree")
#	set(PROGRAMS "${PROGRAMS};testVectorMap")
	set(PROGRAMS "${PROGRAMS};testVectorValues")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
		add_test(${program} ${program})
	endforeach(program)
endif ()

if (BUILD_TIMINGS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};timeFactorOverhead")
	set(PROGRAMS "${PROGRAMS};timeGaussianFactor")
	set(PROGRAMS "${PROGRAMS};timeSLAMlike")
#	set(PROGRAMS "${PROGRAMS};timeVectorValues")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
	endforeach(program)
endif ()



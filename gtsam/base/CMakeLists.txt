CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

SET(lib_target "base")
FILE(GLOB base_src "./*.cpp")

############################# COMPILE LIBRARY
ADD_LIBRARY(${lib_target} STATIC ${base_src})
#TARGET_LINK_LIBRARIES(${lib_target})

############################# INSTALL

#message(STATUS "install ${lib_target} to ${CMAKE_SOURCE_DIR}/lib") 
#install(TARGETS ${lib_target}
#	    ARCHIVE DESTINATION lib COMPONENT dev
	    #LIBRARY DESTINATION lib COMPONENT dev)

# TESTS

set(libs_support ${libs_support} CppUnitLite)

if (BUILD_TESTS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};testBTree")
	set(PROGRAMS "${PROGRAMS};testCholesky")
	set(PROGRAMS "${PROGRAMS};testDebug")
	set(PROGRAMS "${PROGRAMS};testDenseQRUtil")
	set(PROGRAMS "${PROGRAMS};testDSFVector")
	set(PROGRAMS "${PROGRAMS};testFixedVector")
	set(PROGRAMS "${PROGRAMS};testLieScalar")
	set(PROGRAMS "${PROGRAMS};testLieVector")
	set(PROGRAMS "${PROGRAMS};testMatrix")
	set(PROGRAMS "${PROGRAMS};testNumericalDerivative")
	set(PROGRAMS "${PROGRAMS};testTestableAssertions")
	set(PROGRAMS "${PROGRAMS};testVector")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
		add_test(${program} ${program})
	endforeach(program)
endif ()

if (BUILD_TIMINGS)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};timeMatrix")
	set(PROGRAMS "${PROGRAMS};timeublas")
	set(PROGRAMS "${PROGRAMS};timeTest")
	set(PROGRAMS "${PROGRAMS};timeVirtual")
	foreach(program ${PROGRAMS})
		add_executable(${program} "tests/${program}.cpp")
		target_link_libraries(${program} ${lib_target} ${libs_support})
	endforeach(program)
endif ()



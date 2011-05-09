CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

set(libs_support ${libs_support} CppUnitLite ${libs_gtsam})

if (BUILD_EXAMPLES)
	set(PROGRAMS "")
	set(PROGRAMS "${PROGRAMS};SimpleRotation")
	set(PROGRAMS "${PROGRAMS};PlanarSLAMExample_easy")
	set(PROGRAMS "${PROGRAMS};PlanarSLAMSelfContained_advanced")
	set(PROGRAMS "${PROGRAMS};Pose2SLAMExample_easy")
	set(PROGRAMS "${PROGRAMS};Pose2SLAMExample_advanced")
#	set(PROGRAMS "${PROGRAMS};Pose2SLAMwSPCG_easy")
#	set(PROGRAMS "${PROGRAMS};Pose2SLAMwSPCG_advanced")
	foreach(program ${PROGRAMS})
		add_executable(${program} "${program}.cpp")
		target_link_libraries(${program} ${libs_support})
	endforeach(program)
endif ()



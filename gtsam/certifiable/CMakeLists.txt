# Examples for certifiable optimization experiments. These targets are kept
# outside the core gtsam library so SDP prototyping does not change libgtsam.

add_library(gtsam_certifiable STATIC LiftedSDPProblem.cpp LiftedSDPProblem.h)
target_link_libraries(gtsam_certifiable PUBLIC gtsam)
gtsam_apply_build_flags(gtsam_certifiable)
set_property(TARGET gtsam_certifiable PROPERTY FOLDER "certifiable")

if(GTSAM_BUILD_TESTS)
  add_executable(testLiftedSDPs tests/testLiftedSDPs.cpp)
  target_link_libraries(testLiftedSDPs CppUnitLite gtsam)
  gtsam_apply_build_flags(testLiftedSDPs)
  add_test(NAME testLiftedSDPs COMMAND testLiftedSDPs)
  set_property(TARGET testLiftedSDPs PROPERTY FOLDER "tests")
endif()

if(GTSAM_USE_MOSEK)
  target_link_libraries(gtsam_certifiable PRIVATE ${MOSEK_LIBRARIES})
  target_include_directories(gtsam_certifiable SYSTEM PRIVATE ${MOSEK_INCLUDE_DIRS})

  add_executable(Rot2RingQcqpToMonolithicSDP examples/Rot2RingQcqpToMonolithicSDP.cpp)
  target_link_libraries(Rot2RingQcqpToMonolithicSDP gtsam_certifiable)
  gtsam_apply_build_flags(Rot2RingQcqpToMonolithicSDP)
  add_dependencies(examples Rot2RingQcqpToMonolithicSDP)
  set_property(TARGET Rot2RingQcqpToMonolithicSDP PROPERTY FOLDER "examples")

  if(NOT GTSAM_BUILD_EXAMPLES_ALWAYS)
    set_target_properties(Rot2RingQcqpToMonolithicSDP PROPERTIES EXCLUDE_FROM_ALL ON)
  endif()

  add_executable(SimpleMosekSDPExample tests/testMOSEKInstall.cpp)
  target_link_libraries(SimpleMosekSDPExample gtsam ${MOSEK_LIBRARIES})
  target_include_directories(SimpleMosekSDPExample SYSTEM PRIVATE ${MOSEK_INCLUDE_DIRS})
  set_target_properties(SimpleMosekSDPExample PROPERTIES
      BUILD_RPATH "${MOSEK_LIBRARY_DIR}"
      INSTALL_RPATH "${MOSEK_LIBRARY_DIR}")
  gtsam_apply_build_flags(SimpleMosekSDPExample)
  add_dependencies(examples SimpleMosekSDPExample)
  set_property(TARGET SimpleMosekSDPExample PROPERTY FOLDER "examples")

  if(NOT GTSAM_BUILD_EXAMPLES_ALWAYS)
    set_target_properties(SimpleMosekSDPExample PROPERTIES EXCLUDE_FROM_ALL ON)
  endif()
endif()

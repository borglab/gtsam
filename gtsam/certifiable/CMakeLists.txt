# Examples for certifiable optimization experiments. These targets are kept
# outside the core gtsam library so SDP prototyping does not change libgtsam.

add_executable(Rot2RingQcqpExample Rot2RingQcqpExample.cpp)
target_link_libraries(Rot2RingQcqpExample gtsam)
gtsam_apply_build_flags(Rot2RingQcqpExample)
add_dependencies(examples Rot2RingQcqpExample)
set_property(TARGET Rot2RingQcqpExample PROPERTY FOLDER "examples")

if(NOT GTSAM_BUILD_EXAMPLES_ALWAYS)
  set_target_properties(Rot2RingQcqpExample PROPERTIES EXCLUDE_FROM_ALL ON)
endif()

if(GTSAM_USE_MOSEK)
  add_executable(SimpleMosekSDPExample SimpleMosekSDPExample.cpp)
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

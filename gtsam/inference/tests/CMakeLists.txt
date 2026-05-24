# Add standard inference tests. The MOSEK smoke test is gated separately
# because MOSEK is an optional dependency and requires a license to solve.
gtsamAddTestsGlob(inference "test*.cpp" "testMosekSDP.cpp" "gtsam")

if(GTSAM_BUILD_TESTS AND GTSAM_USE_MOSEK)
  add_executable(testMosekSDP testMosekSDP.cpp)
  target_link_libraries(testMosekSDP CppUnitLite gtsam ${MOSEK_LIBRARIES})
  target_include_directories(testMosekSDP SYSTEM PRIVATE ${MOSEK_INCLUDE_DIRS})
  set_target_properties(testMosekSDP PROPERTIES
      BUILD_RPATH "${MOSEK_LIBRARY_DIR}"
      INSTALL_RPATH "${MOSEK_LIBRARY_DIR}")

  gtsam_apply_build_flags(testMosekSDP)
  add_test(NAME testMosekSDP COMMAND testMosekSDP)
  if(MOSEK_LICENSE_FILE)
    set_tests_properties(testMosekSDP PROPERTIES
        ENVIRONMENT "MOSEKLM_LICENSE_FILE=${MOSEK_LICENSE_FILE}")
  endif()
  add_dependencies(check.inference testMosekSDP)
  add_dependencies(check testMosekSDP)
  add_dependencies(all.tests testMosekSDP)
endif()

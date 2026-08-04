set(certifiable_excluded_tests)
set(certifiable_test_libraries gtsam)

if(GTSAM_USE_MOSEK)
  list(APPEND certifiable_test_libraries gtsam_mosek_backend)
else()
  list(APPEND certifiable_excluded_tests "testMosekSDP.cpp")
endif()

gtsamAddTestsGlob(certifiable "test*.cpp" "${certifiable_excluded_tests}"
                  "${certifiable_test_libraries}")

if(GTSAM_USE_MOSEK AND MOSEK_LICENSE_FILE)
  if(GTSAM_SINGLE_TEST_EXE)
    set_tests_properties(check_certifiable_program PROPERTIES
        ENVIRONMENT "MOSEKLM_LICENSE_FILE=${MOSEK_LICENSE_FILE}")
  else()
    set_tests_properties(testMosekSDP testLiftedSDPs PROPERTIES
        ENVIRONMENT "MOSEKLM_LICENSE_FILE=${MOSEK_LICENSE_FILE}")
  endif()
endif()

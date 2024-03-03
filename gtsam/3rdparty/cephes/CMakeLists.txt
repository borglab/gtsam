cmake_minimum_required(VERSION 3.10)
enable_testing()
project(
  cephes
  DESCRIPTION "Cephes Mathematical Function Library"
  VERSION 1.0.0
  LANGUAGES C)

set(CEPHES_HEADER_FILES
    cephes.h
    cephes/dd_idefs.h
    cephes/dd_real.h
    cephes/dd_real_idefs.h
    cephes/expn.h
    cephes/igam.h
    cephes/lanczos.h
    cephes/mconf.h
    cephes/polevl.h
    cephes/sf_error.h)

# Add header files
install(FILES ${CEPHES_HEADER_FILES} DESTINATION include/gtsam/3rdparty/cephes)

set(CEPHES_SOURCES
    cephes/airy.c
    cephes/bdtr.c
    cephes/besselpoly.c
    cephes/beta.c
    cephes/btdtr.c
    cephes/cbrt.c
    cephes/chbevl.c
    cephes/chdtr.c
    cephes/const.c
    cephes/dawsn.c
    cephes/dd_real.c
    cephes/ellie.c
    cephes/ellik.c
    cephes/ellpe.c
    cephes/ellpj.c
    cephes/ellpk.c
    cephes/erfinv.c
    cephes/exp10.c
    cephes/exp2.c
    cephes/expn.c
    cephes/fdtr.c
    cephes/fresnl.c
    cephes/gamma.c
    cephes/gammasgn.c
    cephes/gdtr.c
    cephes/hyp2f1.c
    cephes/hyperg.c
    cephes/i0.c
    cephes/i1.c
    cephes/igam.c
    cephes/igami.c
    cephes/incbet.c
    cephes/incbi.c
    cephes/j0.c
    cephes/j1.c
    cephes/jv.c
    cephes/k0.c
    cephes/k1.c
    cephes/kn.c
    cephes/kolmogorov.c
    cephes/lanczos.c
    cephes/nbdtr.c
    cephes/ndtr.c
    cephes/ndtri.c
    cephes/owens_t.c
    cephes/pdtr.c
    cephes/poch.c
    cephes/psi.c
    cephes/rgamma.c
    cephes/round.c
    cephes/sf_error.c
    cephes/shichi.c
    cephes/sici.c
    cephes/sindg.c
    cephes/sinpi.c
    cephes/spence.c
    cephes/stdtr.c
    cephes/tandg.c
    cephes/tukey.c
    cephes/unity.c
    cephes/yn.c
    cephes/yv.c
    cephes/zeta.c
    cephes/zetac.c)

# Add library source files
add_library(cephes-gtsam SHARED ${CEPHES_SOURCES})

# Add include directory (aka headers)
target_include_directories(
  cephes-gtsam BEFORE PUBLIC $<INSTALL_INTERFACE:include/gtsam/3rdparty/cephes/>
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

set_target_properties(
  cephes-gtsam
  PROPERTIES VERSION ${PROJECT_VERSION}
             SOVERSION ${PROJECT_VERSION_MAJOR}
             C_STANDARD 99)

if(WIN32)
  set_target_properties(
    cephes-gtsam
    PROPERTIES PREFIX ""
               COMPILE_FLAGS /w
               RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/../../../bin")
endif()

if(APPLE)
  set_target_properties(cephes-gtsam PROPERTIES INSTALL_NAME_DIR
                                                "${CMAKE_INSTALL_PREFIX}/lib")
endif()

install(
  TARGETS cephes-gtsam
  EXPORT GTSAM-exports
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

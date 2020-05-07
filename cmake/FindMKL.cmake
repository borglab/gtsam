# This file is adapted from the one in OpenMEEG: http://www-sop.inria.fr/athena/software/OpenMEEG/
# - Try to find the Intel Math Kernel Library
# Once done this will define
#
#  MKL_FOUND - system has MKL
#  MKL_ROOT_DIR - path to the MKL base directory
#  MKL_INCLUDE_DIR - the MKL include directory
#  MKL_LIBRARIES - MKL libraries
#
# There are few sets of libraries:
# Array indexes modes:
# LP - 32 bit indexes of arrays
# ILP - 64 bit indexes of arrays
# Threading:
# SEQUENTIAL - no threading
# INTEL - Intel threading library
# GNU - GNU threading library
# MPI support
# NOMPI - no MPI support
# INTEL - Intel MPI library
# OPEN - Open MPI library
# SGI - SGI MPT Library

# linux
IF(UNIX AND NOT APPLE)
    IF(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
        SET(MKL_ARCH_DIR "intel64")
    ELSE()
        SET(MKL_ARCH_DIR "32")
    ENDIF()
ENDIF()
# macos
IF(APPLE)
    SET(MKL_ARCH_DIR "intel64")
ENDIF()

IF(FORCE_BUILD_32BITS)
    set(MKL_ARCH_DIR "32")
ENDIF()
# windows
IF(WIN32)
    IF(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
        SET(MKL_ARCH_DIR "intel64")
    ELSE()
        SET(MKL_ARCH_DIR "ia32")
    ENDIF()
ENDIF()

SET(MKL_THREAD_VARIANTS SEQUENTIAL GNUTHREAD INTELTHREAD)
SET(MKL_MODE_VARIANTS ILP LP)
SET(MKL_MPI_VARIANTS NOMPI INTELMPI OPENMPI SGIMPT)

FIND_PATH(MKL_ROOT_DIR
    include/mkl_cblas.h
    PATHS
    $ENV{MKLDIR}
    /opt/intel/mkl/
    /opt/intel/mkl/*/
    /opt/intel/cmkl/
    /opt/intel/cmkl/*/
    /opt/intel/*/mkl/
    /Library/Frameworks/Intel_MKL.framework/Versions/Current/lib/universal
        "C:/Program Files (x86)/Intel/ComposerXE-2011/mkl"
        "C:/Program Files (x86)/Intel/Composer XE 2013/mkl"
        "C:/Program Files/Intel/MKL/*/"
        "C:/Program Files/Intel/ComposerXE-2011/mkl"
        "C:/Program Files/Intel/Composer XE 2013/mkl"
)

FIND_PATH(MKL_INCLUDE_DIR
  mkl_cblas.h
  PATHS
    ${MKL_ROOT_DIR}/include
    ${INCLUDE_INSTALL_DIR}
)

FIND_PATH(MKL_FFTW_INCLUDE_DIR
  fftw3.h
  PATH_SUFFIXES fftw
  PATHS
    ${MKL_ROOT_DIR}/include
    ${INCLUDE_INSTALL_DIR}
  NO_DEFAULT_PATH
)

IF(WIN32 AND MKL_ROOT_DIR)
        SET(MKL_LIB_SEARCHPATH $ENV{ICC_LIB_DIR} $ENV{MKL_LIB_DIR} "${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}" "${MKL_ROOT_DIR}/../compiler" "${MKL_ROOT_DIR}/../compiler/lib/${MKL_ARCH_DIR}")
        IF(MKL_INCLUDE_DIR MATCHES "2017" OR MKL_INCLUDE_DIR MATCHES "2018")
                IF(CMAKE_CL_64)
                        SET(MKL_LIBS mkl_core mkl_intel_lp64 mkl_lapack95_lp64 mkl_blas95_lp64)
                ELSE()
                        SET(MKL_LIBS mkl_core mkl_intel_s mkl_lapack95 mkl_blas95)
                ENDIF()
                IF(TBB_FOUND AND GTSAM_WITH_TBB)
                        SET(MKL_LIBS ${MKL_LIBS} mkl_tbb_thread)
                ELSE()
                        SET(MKL_LIBS ${MKL_LIBS} mkl_intel_thread libiomp5md)
                ENDIF()
        ELSEIF(MKL_INCLUDE_DIR MATCHES "10.")
                IF(CMAKE_CL_64)
                        SET(MKL_LIBS mkl_solver_lp64 mkl_core mkl_intel_lp64 mkl_intel_thread libguide mkl_lapack95_lp64 mkl_blas95_lp64)
                ELSE()
                        SET(MKL_LIBS mkl_solver mkl_core mkl_intel_c mkl_intel_s mkl_intel_thread libguide mkl_lapack95 mkl_blas95)
                ENDIF()
        ELSEIF(MKL_INCLUDE_DIR MATCHES "2013") # version 11 ...
                IF(CMAKE_CL_64)
                        SET(MKL_LIBS mkl_core mkl_intel_lp64 mkl_intel_thread libiomp5md mkl_lapack95_lp64 mkl_blas95_lp64)
                ELSE()
                        SET(MKL_LIBS mkl_core mkl_intel_c mkl_intel_s mkl_intel_thread libiomp5md mkl_lapack95 mkl_blas95)
                ENDIF()
        ELSE() # old MKL 9
                SET(MKL_LIBS mkl_solver mkl_c libguide mkl_lapack mkl_ia32)
        ENDIF()

        IF (MKL_INCLUDE_DIR MATCHES "10.3")
                SET(MKL_LIBS ${MKL_LIBS} libiomp5md)
        ENDIF()

        FOREACH (LIB ${MKL_LIBS})
                FIND_LIBRARY(${LIB}_PATH ${LIB} PATHS ${MKL_LIB_SEARCHPATH} ENV LIBRARY_PATH)
                IF(${LIB}_PATH)
                        SET(MKL_LIBRARIES ${MKL_LIBRARIES} ${${LIB}_PATH})
                ELSE()
                        MESSAGE(STATUS "Could not find ${LIB}: disabling MKL")
                ENDIF()
        ENDFOREACH()
        SET(MKL_FOUND ON)
ELSEIF(MKL_ROOT_DIR) # UNIX and macOS
        FIND_LIBRARY(MKL_CORE_LIBRARY
          mkl_core
          PATHS
                ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                ${MKL_ROOT_DIR}/lib/
        )

        # Threading libraries
        FIND_LIBRARY(MKL_SEQUENTIAL_LIBRARY
          mkl_sequential
          PATHS
                ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                ${MKL_ROOT_DIR}/lib/
        )

        FIND_LIBRARY(MKL_INTELTHREAD_LIBRARY
          mkl_intel_thread
          PATHS
                ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                ${MKL_ROOT_DIR}/lib/
        )

        # MKL on Mac OS doesn't ship with GNU thread versions, only Intel versions (see above)
        IF(NOT APPLE)
            FIND_LIBRARY(MKL_GNUTHREAD_LIBRARY
              mkl_gnu_thread
              PATHS
                    ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                    ${MKL_ROOT_DIR}/lib/
            )
        ENDIF()

        # Intel Libraries
        IF("${MKL_ARCH_DIR}" STREQUAL "32")
                FIND_LIBRARY(MKL_LP_LIBRARY
                  mkl_intel
                  PATHS
                        ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                        ${MKL_ROOT_DIR}/lib/
                )

                FIND_LIBRARY(MKL_ILP_LIBRARY
                  mkl_intel
                  PATHS
                        ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                        ${MKL_ROOT_DIR}/lib/
                )
        else()
                FIND_LIBRARY(MKL_LP_LIBRARY
                  mkl_intel_lp64
                  PATHS
                        ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                        ${MKL_ROOT_DIR}/lib/
                )

                FIND_LIBRARY(MKL_ILP_LIBRARY
                  mkl_intel_ilp64
                  PATHS
                        ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                        ${MKL_ROOT_DIR}/lib/
                )
        ENDIF()

        # Lapack
        FIND_LIBRARY(MKL_LAPACK_LIBRARY
          mkl_lapack
          PATHS
                ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                ${MKL_ROOT_DIR}/lib/
        )

        IF(NOT MKL_LAPACK_LIBRARY)
                FIND_LIBRARY(MKL_LAPACK_LIBRARY
                  mkl_lapack95_lp64
                  PATHS
                        ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                        ${MKL_ROOT_DIR}/lib/
                )
        ENDIF()

        IF(NOT MKL_LAPACK_LIBRARY)
                FIND_LIBRARY(MKL_LAPACK_LIBRARY
                  mkl_intel_lp64
                  PATHS
                        ${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}
                        ${MKL_ROOT_DIR}/lib/
                )
        ENDIF()

        # iomp5
        IF("${MKL_ARCH_DIR}" STREQUAL "32")
                IF(UNIX AND NOT APPLE)
                        FIND_LIBRARY(MKL_IOMP5_LIBRARY
                          iomp5
                          PATHS
                                ${MKL_ROOT_DIR}/../lib/ia32
                        )
                ELSE()
                        SET(MKL_IOMP5_LIBRARY "") # no need for mac
                ENDIF()
        else()
                IF(UNIX AND NOT APPLE)
                        FIND_LIBRARY(MKL_IOMP5_LIBRARY
                          iomp5
                          PATHS
                                ${MKL_ROOT_DIR}/lib/intel64
                                ${MKL_ROOT_DIR}/../lib/intel64
                        )
                ELSE()
                        SET(MKL_IOMP5_LIBRARY "") # no need for mac
                ENDIF()
        ENDIF()

        foreach (MODEVAR ${MKL_MODE_VARIANTS})
                foreach (THREADVAR ${MKL_THREAD_VARIANTS})
                        if (MKL_CORE_LIBRARY AND MKL_${MODEVAR}_LIBRARY AND MKL_${THREADVAR}_LIBRARY)
                                set(MKL_${MODEVAR}_${THREADVAR}_LIBRARIES
                                        ${MKL_${MODEVAR}_LIBRARY} ${MKL_${THREADVAR}_LIBRARY} ${MKL_CORE_LIBRARY}
                                        ${MKL_LAPACK_LIBRARY} ${MKL_IOMP5_LIBRARY})
                                # message("${MODEVAR} ${THREADVAR} ${MKL_${MODEVAR}_${THREADVAR}_LIBRARIES}") # for debug
                        endif()
                endforeach()
        endforeach()

        IF(APPLE)
            SET(MKL_LIBRARIES ${MKL_LP_INTELTHREAD_LIBRARIES})
        ELSE()
            SET(MKL_LIBRARIES ${MKL_LP_GNUTHREAD_LIBRARIES})
        ENDIF()

        MARK_AS_ADVANCED(MKL_CORE_LIBRARY MKL_LP_LIBRARY MKL_ILP_LIBRARY
                MKL_SEQUENTIAL_LIBRARY MKL_INTELTHREAD_LIBRARY MKL_GNUTHREAD_LIBRARY)
ENDIF()

INCLUDE(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MKL DEFAULT_MSG MKL_INCLUDE_DIR MKL_LIBRARIES)

#if(MKL_FOUND)
#        LINK_DIRECTORIES(${MKL_ROOT_DIR}/lib/${MKL_ARCH_DIR}) # hack
#endif()

MARK_AS_ADVANCED(MKL_INCLUDE_DIR MKL_LIBRARIES)

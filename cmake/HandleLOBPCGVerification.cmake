# LOBPCG verification (Optimization + Preconditioners::ILDL).
#
# We do not add_subdirectory(Optimization|Preconditioners) — their CMake
# transitively requires SPQR (used only by LSChol, which we do not build)
# and disagrees with install(EXPORT). Compiling ILDL sources straight into
# libgtsam avoids the gymnastics.

option(GTSAM_USE_LOBPCG_VERIFICATION
  "Build with LOBPCG-based certificate verification in the Riemannian Staircase. Requires libsuitesparse-dev (CHOLMOD) and a BLAS implementation."
  ON)

if(NOT GTSAM_USE_LOBPCG_VERIFICATION)
  return()
endif()

find_library(GTSAM_CHOLMOD_LIB cholmod REQUIRED)
find_path(GTSAM_CHOLMOD_INCLUDE_DIR cholmod.h
  PATHS /usr/include/suitesparse /usr/local/include/suitesparse
  REQUIRED)
find_package(BLAS REQUIRED)

set(GTSAM_LOBPCG_SOURCES
  ${GTSAM_SOURCE_DIR}/gtsam/3rdparty/Preconditioners/Preconditioners/ILDL/src/ILDL.cpp
  ${GTSAM_SOURCE_DIR}/gtsam/3rdparty/Preconditioners/Preconditioners/ILDL/src/ILDL_utils.cpp
  CACHE INTERNAL "")

set(GTSAM_LOBPCG_INCLUDE_DIRS
  ${GTSAM_SOURCE_DIR}/gtsam/3rdparty/Optimization/include
  # ILDL.h does `#include "Preconditioners/Types.h"`.
  ${GTSAM_SOURCE_DIR}/gtsam/3rdparty/Preconditioners
  ${GTSAM_SOURCE_DIR}/gtsam/3rdparty/Preconditioners/Preconditioners/ILDL/include
  ${GTSAM_SOURCE_DIR}/gtsam/3rdparty/Preconditioners/SymILDL/SymILDL
  ${GTSAM_CHOLMOD_INCLUDE_DIR}
  CACHE INTERNAL "")

set(GTSAM_LOBPCG_LINK_LIBRARIES
  ${GTSAM_CHOLMOD_LIB}
  ${BLAS_LIBRARIES}
  CACHE INTERNAL "")

# Quiet upstream ILDL warnings that gtsam's -Werror would otherwise reject.
set(GTSAM_LOBPCG_COMPILE_OPTIONS
  -Wno-sign-compare
  -Wno-unused-parameter
  -Wno-unused-but-set-variable
  CACHE INTERNAL "")

message(STATUS "LOBPCG verification: ENABLED (cholmod: ${GTSAM_CHOLMOD_LIB})")

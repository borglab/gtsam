###############################################################################
# Find SuiteSparse
find_package(SuiteSparse COMPONENTS CHOLMOD)

if(CHOLMOD_FOUND AND GTSAM_WITH_SUITESPARSE)
  set(GTSAM_USE_SUITESPARSE 1) # This will go into config.h
  message(STATUS "Found CHOLMOD at ${CHOLMOD_LIBRARY}")
  list(APPEND GTSAM_ADDITIONAL_LIBRARIES ${CHOLMOD_LIBRARY})
else()
  set(GTSAM_USE_SUITESPARSE 0)
endif()

# Add this directory for internal users.
include_directories(.)
# Find sources.
file(GLOB metis_sources *.c)
# Build libmetis.
add_library(metis ${METIS_LIBRARY_TYPE} ${GKlib_sources} ${metis_sources})
if(UNIX)
  target_link_libraries(metis m)
endif()


install(TARGETS metis
  LIBRARY DESTINATION include/gtsam/3rdparty/metis/lib
  RUNTIME DESTINATION include/gtsam/3rdparty/metis/lib
  ARCHIVE DESTINATION include/gtsam/3rdparty/metis/lib)

install(TARGETS metis EXPORT GTSAM-exports ARCHIVE DESTINATION lib)

list(APPEND GTSAM_EXPORTED_TARGETS metis)
set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)

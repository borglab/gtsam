# Add this directory for internal users.
include_directories(.)
# Find sources.
file(GLOB metis_sources *.c)
# Build libmetis.
add_library(metis-gtsam ${METIS_LIBRARY_TYPE} ${GKlib_sources} ${metis_sources})
if(UNIX)
  target_link_libraries(metis-gtsam m)
endif()

if(WIN32)
		set_target_properties(metis-gtsam PROPERTIES
			COMPILE_FLAGS /w
			WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

if (APPLE)
		set_target_properties(metis-gtsam PROPERTIES
			INSTALL_NAME_DIR
			"${CMAKE_INSTALL_PREFIX}/lib")
 endif()

install(TARGETS metis-gtsam EXPORT GTSAM-exports
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
list(APPEND GTSAM_EXPORTED_TARGETS metis-gtsam)
set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)

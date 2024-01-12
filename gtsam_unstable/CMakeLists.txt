project(gtsam_unstable LANGUAGES CXX)

# Build full gtsam_unstable library as a single library
# and also build tests
set (gtsam_unstable_subdirs
    base
    geometry
    linear
    discrete
    dynamics
    nonlinear
    slam
)

if(GTSAM_SUPPORT_NESTED_DISSECTION) # Only build partition if metis is built
    set (gtsam_unstable_subdirs ${gtsam_unstable_subdirs} partition)
endif(GTSAM_SUPPORT_NESTED_DISSECTION)

add_custom_target(check.unstable COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

# To exclude a source from the library build (in any subfolder)
# Add the full name to this list, as in the following example
# Sources to remove from builds
set (excluded_sources # "")
    "${CMAKE_CURRENT_SOURCE_DIR}/slam/serialization.cpp"
)

set (excluded_headers # "")
    "${CMAKE_CURRENT_SOURCE_DIR}/slam/serialization.h"
)

# if GTSAM_USE_BOOST_FEATURES is not set, then we need to exclude the following:
if(NOT GTSAM_USE_BOOST_FEATURES)
    list (APPEND excluded_sources
        "${CMAKE_CURRENT_SOURCE_DIR}/linear/QPSParser.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/linear/QPSSolver.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/discrete/Scheduler.cpp"
    )
    list (APPEND excluded_headers
        "${CMAKE_CURRENT_SOURCE_DIR}/linear/QPSParser.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/linear/QPSSolver.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/discrete/Scheduler.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/parition/FindSeparator.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/parition/FindSeparator-inl.h"
    )
endif()

# assemble core libraries
foreach(subdir ${gtsam_unstable_subdirs})
    # Build convenience libraries
    file(GLOB subdir_srcs "${subdir}/*.cpp")
	file(GLOB subdir_headers "${subdir}/*.h")
    list(REMOVE_ITEM subdir_srcs ${excluded_sources})
    list(REMOVE_ITEM subdir_headers ${excluded_headers})
    set(${subdir}_srcs ${subdir_srcs} ${subdir_headers})
    gtsam_assign_source_folders("${${subdir}_srcs}") # Create MSVC structure

    # Build local library and tests
    message(STATUS "Building ${subdir}_unstable")
    add_subdirectory(${subdir})
endforeach(subdir)

# dllexport.h
set(library_name GTSAM_UNSTABLE) # For substitution in dllexport.h.in
configure_file("${GTSAM_SOURCE_DIR}/cmake/dllexport.h.in" "dllexport.h")
list(APPEND gtsam_unstable_srcs "${PROJECT_BINARY_DIR}/dllexport.h")
install(FILES "${PROJECT_BINARY_DIR}/dllexport.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable)

# assemble gtsam_unstable components
set(gtsam_unstable_srcs
    ${base_srcs}
    ${geometry_srcs}
    ${linear_srcs}
    ${discrete_srcs}
    ${dynamics_srcs}
    ${nonlinear_srcs}
    ${slam_srcs}
)

if(GTSAM_SUPPORT_NESTED_DISSECTION) # Only build partition if metis is built
    set (gtsam_unstable_srcs ${gtsam_unstable_srcs} ${partition_srcs})
endif(GTSAM_SUPPORT_NESTED_DISSECTION)

# Versions - same as core gtsam library
set(gtsam_unstable_version   ${GTSAM_VERSION_STRING})
set(gtsam_unstable_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM_UNSTABLE Version: ${gtsam_unstable_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

add_library(gtsam_unstable ${GTSAM_LIBRARY_TYPE} ${gtsam_unstable_srcs})

# Apply build flags:
gtsam_apply_build_flags(gtsam_unstable)

set_target_properties(gtsam_unstable PROPERTIES
  OUTPUT_NAME         gtsam_unstable
  CLEAN_DIRECT_OUTPUT 1
  VERSION             ${gtsam_unstable_version}
  SOVERSION           ${gtsam_unstable_soversion})
target_link_libraries(gtsam_unstable PUBLIC gtsam)
# No need to link against Boost here, it's inherited from gtsam PUBLIC interface

if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
	if (NOT GTSAM_SHARED_LIB)
		set_target_properties(gtsam_unstable PROPERTIES
			PREFIX "lib"
			COMPILE_DEFINITIONS GTSAM_UNSTABLE_IMPORT_STATIC)
	else()
		set_target_properties(gtsam_unstable PROPERTIES
			PREFIX ""
			DEFINE_SYMBOL GTSAM_UNSTABLE_EXPORTS
			RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
	endif()
endif()

install(
	TARGETS gtsam_unstable
	EXPORT GTSAM_UNSTABLE-exports
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
list(APPEND GTSAM_UNSTABLE_EXPORTED_TARGETS gtsam_unstable)
set(GTSAM_UNSTABLE_EXPORTED_TARGETS "${GTSAM_UNSTABLE_EXPORTED_TARGETS}" PARENT_SCOPE)

# Build examples
add_subdirectory(examples)

# Build timing
add_subdirectory(timing)

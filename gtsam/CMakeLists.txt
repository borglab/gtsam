# We split the library in to separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add
set (gtsam_subdirs
    base
    geometry
    inference
    symbolic
    discrete
    linear
    nonlinear
    sam
    sfm
    slam
    smart
	navigation
)

set(gtsam_srcs)

# Build 3rdparty separately
message(STATUS "Building 3rdparty")
add_subdirectory(3rdparty)

set (3rdparty_srcs
 ${eigen_headers} # Set by 3rdparty/CMakeLists.txt
 ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/CCOLAMD/Source/ccolamd.c
 ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/SuiteSparse_config/SuiteSparse_config.c)
gtsam_assign_source_folders("${3rdparty_srcs}") # Create MSVC structure

# To exclude a source from the library build (in any subfolder)
# Add the full name to this list, as in the following example
# Sources to remove from builds
set (excluded_sources #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/slam/serialization.cpp"
)

set (excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/slam/serialization.h"
)

if(GTSAM_USE_QUATERNIONS)
    set(excluded_sources ${excluded_sources} "${CMAKE_CURRENT_SOURCE_DIR}/geometry/Rot3M.cpp")
else()
    set(excluded_sources ${excluded_sources} "${CMAKE_CURRENT_SOURCE_DIR}/geometry/Rot3Q.cpp")
endif()

# Common headers
file(GLOB gtsam_core_headers "*.h")
install(FILES ${gtsam_core_headers} DESTINATION include/gtsam)

# assemble core libaries
foreach(subdir ${gtsam_subdirs})
    # Build convenience libraries
    file(GLOB_RECURSE subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h") # Include header files so they show up in Visual Studio
    list(REMOVE_ITEM subdir_srcs ${excluded_sources})
	file(GLOB subdir_test_files "${subdir}/tests/*")
	list(REMOVE_ITEM subdir_srcs ${subdir_test_files}) # Remove test files from sources compiled into library
    gtsam_assign_source_folders("${subdir_srcs}") # Create MSVC structure
    set(${subdir}_srcs ${subdir_srcs})

    # Build local library and tests
    message(STATUS "Building ${subdir}")
    add_subdirectory(${subdir})
endforeach(subdir)

# To add additional sources to gtsam when building the full library (static or shared)
# Add the subfolder with _srcs appended to the end to this list
set(gtsam_srcs
    ${3rdparty_srcs}
    ${base_srcs}
    ${geometry_srcs}
    ${inference_srcs}
    ${symbolic_srcs}
    ${discrete_srcs}
    ${linear_srcs}
    ${nonlinear_srcs}
    ${slam_srcs}
    ${navigation_srcs}
	${gtsam_core_headers}
)

# Generate and install config and dllexport files
configure_file(config.h.in config.h)
set(library_name GTSAM) # For substitution in dllexport.h.in
configure_file("${PROJECT_SOURCE_DIR}/cmake/dllexport.h.in" "dllexport.h")
list(APPEND gtsam_srcs "${PROJECT_BINARY_DIR}/gtsam/config.h" "${PROJECT_BINARY_DIR}/gtsam/dllexport.h")
install(FILES "${PROJECT_BINARY_DIR}/gtsam/config.h" "${PROJECT_BINARY_DIR}/gtsam/dllexport.h" DESTINATION include/gtsam)

if(GTSAM_SUPPORT_NESTED_DISSECTION)
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES metis)
endif()

# Versions
set(gtsam_version   ${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH})
set(gtsam_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM Version: ${gtsam_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# build shared and static versions of the library
if (GTSAM_BUILD_STATIC_LIBRARY)
    message(STATUS "Building GTSAM - static")
    add_library(gtsam STATIC ${gtsam_srcs})
	target_link_libraries(gtsam ${GTSAM_BOOST_LIBRARIES} ${GTSAM_ADDITIONAL_LIBRARIES})
    set_target_properties(gtsam PROPERTIES
        OUTPUT_NAME         gtsam
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_version}
     	SOVERSION           ${gtsam_soversion})
	if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
		set_target_properties(gtsam PROPERTIES
			PREFIX "lib"
			COMPILE_DEFINITIONS GTSAM_IMPORT_STATIC)
	endif()
    install(TARGETS gtsam EXPORT GTSAM-exports ARCHIVE DESTINATION lib)
    list(APPEND GTSAM_EXPORTED_TARGETS gtsam)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
else()
    message(STATUS "Building GTSAM - shared")
    add_library(gtsam SHARED ${gtsam_srcs})
    target_link_libraries(gtsam ${GTSAM_BOOST_LIBRARIES} ${GTSAM_ADDITIONAL_LIBRARIES})
    set_target_properties(gtsam PROPERTIES
        OUTPUT_NAME         gtsam
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_version}
     	SOVERSION           ${gtsam_soversion})
	if(WIN32)
		set_target_properties(gtsam PROPERTIES
			PREFIX ""
			DEFINE_SYMBOL GTSAM_EXPORTS
			RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
	endif()
    install(TARGETS gtsam EXPORT GTSAM-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
    list(APPEND GTSAM_EXPORTED_TARGETS gtsam)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
endif()

# make sure that ccolamd compiles even in face of warnings
if(WIN32)
    set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "-w")
else()
    set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "-Wno-error")
endif()

# Special cases
if(MSVC)
	set_property(SOURCE
		"${CMAKE_CURRENT_SOURCE_DIR}/slam/serialization.cpp"
		"${CMAKE_CURRENT_SOURCE_DIR}/nonlinear/ISAM2.cpp"
		APPEND PROPERTY COMPILE_FLAGS "/bigobj")
endif()

# Create the matlab toolbox for the gtsam library
if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Set up codegen
    include(GtsamMatlabWrap)

    # Generate, build and install toolbox
    set(mexFlags "${GTSAM_BUILD_MEX_BINARY_FLAGS}")
	if(GTSAM_BUILD_STATIC_LIBRARY)
		list(APPEND mexFlags -DGTSAM_IMPORT_STATIC)
	endif()

    # Wrap
    wrap_and_install_library(../gtsam.h "${GTSAM_ADDITIONAL_LIBRARIES}" "" "${mexFlags}")
endif ()


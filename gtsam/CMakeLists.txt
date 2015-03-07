# We split the library in to separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add
set (gtsam_subdirs 
    base 
    geometry 
    inference 
    discrete 
    linear 
    nonlinear 
    slam
)

set(gtsam_srcs)

# Build 3rdparty separately
message(STATUS "Building 3rdparty") 
add_subdirectory(3rdparty)

# build convenience library
set (3rdparty_srcs 
 ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/CCOLAMD/Source/ccolamd.c 
 ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/CCOLAMD/Source/ccolamd_global.c
 ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/UFconfig/UFconfig.c)
gtsam_assign_source_folders("${3rdparty_srcs}") # Create MSVC structure
if (GTSAM_BUILD_CONVENIENCE_LIBRARIES)
    message(STATUS "Building Convenience Library: ccolamd")
    add_library(ccolamd STATIC ${3rdparty_srcs})
endif()

# To exclude a source from the library build (in any subfolder)
# Add the full name to this list, as in the following example
# Sources to remove from builds
set (excluded_sources "")
#    "${CMAKE_CURRENT_SOURCE_DIR}/discrete/TypedDiscreteFactor.cpp" 
#    "${CMAKE_CURRENT_SOURCE_DIR}/discrete/TypedDiscreteFactorGraph.cpp" 
#    "${CMAKE_CURRENT_SOURCE_DIR}/discrete/parseUAI.cpp" 
#    "${CMAKE_CURRENT_SOURCE_DIR}/discrete/PotentialTable.cpp")
    
if(GTSAM_USE_QUATERNIONS)
    set(excluded_sources ${excluded_sources} "${CMAKE_CURRENT_SOURCE_DIR}/geometry/Rot3M.cpp")
else()
    set(excluded_sources ${excluded_sources} "${CMAKE_CURRENT_SOURCE_DIR}/geometry/Rot3Q.cpp")
endif()

# assemble core libaries
foreach(subdir ${gtsam_subdirs})
    # Build convenience libraries
    file(GLOB subdir_srcs "${subdir}/*.cpp")
    file(GLOB subdir_headers "${subdir}/*.h")
    set(subdir_srcs ${subdir_srcs} ${subdir_headers}) # Include header files so they show up in Visual Studio
    gtsam_assign_source_folders("${subdir_srcs}") # Create MSVC structure
    list(REMOVE_ITEM subdir_srcs ${excluded_sources})
    set(${subdir}_srcs ${subdir_srcs})
    if (GTSAM_BUILD_CONVENIENCE_LIBRARIES)
        message(STATUS "Building Convenience Library: ${subdir}")
        add_library(${subdir} STATIC ${subdir_srcs})
    endif()

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
    ${discrete_srcs}
    ${linear_srcs}
    ${nonlinear_srcs}
    ${slam_srcs}
)

# Versions
set(gtsam_version   ${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH})
set(gtsam_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM Version: ${gtsam_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
 
# build shared and static versions of the library
if (GTSAM_BUILD_STATIC_LIBRARY)
    message(STATUS "Building GTSAM - static")
    add_library(gtsam-static STATIC ${gtsam_srcs})
	target_link_libraries(gtsam-static ${GTSAM_BOOST_LIBRARIES})
    set_target_properties(gtsam-static PROPERTIES 
        OUTPUT_NAME         gtsam
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_version}
     	SOVERSION           ${gtsam_soversion})
    install(TARGETS gtsam-static EXPORT GTSAM-exports ARCHIVE DESTINATION lib)
    list(APPEND GTSAM_EXPORTED_TARGETS gtsam-static)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
    set(gtsam-lib gtsam-static)
endif (GTSAM_BUILD_STATIC_LIBRARY)

if (GTSAM_BUILD_SHARED_LIBRARY)
    message(STATUS "Building GTSAM - shared")
    add_library(gtsam-shared SHARED ${gtsam_srcs})
	target_link_libraries(gtsam-shared ${GTSAM_BOOST_LIBRARIES})
    set_target_properties(gtsam-shared PROPERTIES 
        OUTPUT_NAME         gtsam
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_version}
     	SOVERSION           ${gtsam_soversion})
    install(TARGETS gtsam-shared EXPORT GTSAM-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
    list(APPEND GTSAM_EXPORTED_TARGETS gtsam-shared)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
    if (NOT GTSAM_BUILD_STATIC_LIBRARY)
        set(gtsam-lib "gtsam-shared")
    endif()
endif(GTSAM_BUILD_SHARED_LIBRARY)

# Set dataset paths
set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/slam/dataset.cpp"
    APPEND PROPERTY COMPILE_DEFINITIONS
	"SOURCE_TREE_DATASET_DIR=\"${CMAKE_SOURCE_DIR}/examples/Data\""
	"INSTALLED_DATASET_DIR=\"${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam_examples/Data\"")

# Create the matlab toolbox for the gtsam library
if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Set up codegen
    include(GtsamMatlabWrap)
    
    # Choose include flags depending on build process
	set(MEX_INCLUDE_ROOT ${GTSAM_SOURCE_ROOT_DIR})
	set(MEX_LIB_ROOT ${CMAKE_BINARY_DIR})
	set(GTSAM_LIB_DIR ${MEX_LIB_ROOT}/gtsam)
    
    # Generate, build and install toolbox
    set(mexFlags ${GTSAM_BUILD_MEX_BINARY_FLAGS} -I${Boost_INCLUDE_DIR} -I${MEX_INCLUDE_ROOT})

    # Macro to handle details of setting up targets
    # FIXME: issue with dependency between wrap_gtsam and wrap_gtsam_build, only shows up on CMake 2.8.3
    wrap_library(gtsam "${mexFlags}" "../" "")
endif ()

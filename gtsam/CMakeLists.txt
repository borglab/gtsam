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
	target_link_libraries(gtsam-static ${Boost_SERIALIZATION_LIBRARY})
    set_target_properties(gtsam-static PROPERTIES 
        OUTPUT_NAME         gtsam
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_version}
     	SOVERSION           ${gtsam_soversion})
	if(MSVC)
	  set_target_properties(gtsam-static PROPERTIES
	      COMPILE_FLAGS "/MP")
    endif()
    install(TARGETS gtsam-static ARCHIVE DESTINATION lib)
endif (GTSAM_BUILD_STATIC_LIBRARY)

if (GTSAM_BUILD_SHARED_LIBRARY)
    message(STATUS "Building GTSAM - shared")
    add_library(gtsam-shared SHARED ${gtsam_srcs})
	target_link_libraries(gtsam-shared ${Boost_SERIALIZATION_LIBRARY})
    set_target_properties(gtsam-shared PROPERTIES 
        OUTPUT_NAME         gtsam
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_version}
     	SOVERSION           ${gtsam_soversion})
	if(MSVC)
	  set_target_properties(gtsam-shared PROPERTIES
	      COMPILE_FLAGS "/MP")
    endif()
    install(TARGETS gtsam-shared LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
endif(GTSAM_BUILD_SHARED_LIBRARY)

# Create the matlab toolbox for the gtsam library
if (GTSAM_BUILD_WRAP)
    # Set up codegen
    include(GtsamMatlabWrap)
    
    # TODO: generate these includes programmatically
    # Choose include flags depending on build process
    if (GTSAM_BUILD_MEX_BIN)
        set(MEX_INCLUDE_ROOT ${GTSAM_SOURCE_ROOT_DIR})
        set(MEX_LIB_ROOT ${CMAKE_BINARY_DIR})
        set(GTSAM_LIB_DIR ${MEX_LIB_ROOT}/gtsam)
    else()
        set(MEX_INCLUDE_ROOT ${CMAKE_INSTALL_PREFIX}/include)
        set(MEX_LIB_ROOT ${CMAKE_INSTALL_PREFIX}/lib)
        set(GTSAM_LIB_DIR ${MEX_LIB_ROOT})
    endif()
    
    # Generate, build and install toolbox
    set(mexFlags "-I${Boost_INCLUDE_DIR} -I${MEX_INCLUDE_ROOT} -I${MEX_INCLUDE_ROOT}/gtsam -I${MEX_INCLUDE_ROOT}/gtsam/base -I${MEX_INCLUDE_ROOT}/gtsam/geometry -I${MEX_INCLUDE_ROOT}/gtsam/linear -I${MEX_INCLUDE_ROOT}/gtsam/discrete -I${MEX_INCLUDE_ROOT}/gtsam/inference -I${MEX_INCLUDE_ROOT}/gtsam/nonlinear -I${MEX_INCLUDE_ROOT}/gtsam/slam -L${GTSAM_LIB_DIR} -lgtsam")
    
    # Macro to handle details of setting up targets
    wrap_library(gtsam "${mexFlags}" "../")
endif ()

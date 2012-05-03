# Build full gtsam2 library as a single library
# and also build tests 
set (gtsam2_subdirs
    base 
    dynamics
    geometry
    slam 
)

# assemble core libaries
foreach(subdir ${gtsam2_subdirs})
    # Build convenience libraries
    file(GLOB subdir_srcs "${subdir}/*.cpp")
    set(${subdir}_srcs ${subdir_srcs})
    if (GTSAM2_BUILD_CONVENIENCE_LIBRARIES)
        message(STATUS "Building Convenience Library: ${subdir}")
        add_library(${subdir} STATIC ${subdir_srcs})
    endif()

    # Build local library and tests
    message(STATUS "Building ${subdir}") 
    add_subdirectory(${subdir})
endforeach(subdir)
 
# assemble gtsam2 components
set(gtsam2_srcs
    ${base_srcs} 
    ${discrete_srcs}
    ${dynamics_srcs}
    ${geometry_srcs}
    ${slam_srcs} 
)

option (GTSAM2_BUILD_SHARED_LIBRARY "Enable/Disable building of a shared version of gtsam2" ON)
 
# Versions
set(gtsam2_version   ${GTSAM2_VERSION_MAJOR}.${GTSAM2_VERSION_MINOR}.${GTSAM2_VERSION_PATCH})
set(gtsam2_soversion ${GTSAM2_VERSION_MAJOR})
message(STATUS "GTSAM2 Version: ${gtsam2_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
 
# build shared and static versions of the library
message(STATUS "Building GTSAM2 - static")
add_library(gtsam2-static STATIC ${gtsam2_srcs})
set_target_properties(gtsam2-static PROPERTIES 
    OUTPUT_NAME         gtsam2
    CLEAN_DIRECT_OUTPUT 1
	VERSION             ${gtsam2_version}
 	SOVERSION           ${gtsam2_soversion})
install(TARGETS gtsam2-static ARCHIVE DESTINATION lib)

if (GTSAM2_BUILD_SHARED_LIBRARY)
    message(STATUS "Building GTSAM2 - shared")
    add_library(gtsam2-shared SHARED ${gtsam2_srcs})
    set_target_properties(gtsam2-shared PROPERTIES 
        OUTPUT_NAME         gtsam2
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam2_version}
     	SOVERSION           ${gtsam2_soversion})
    install(TARGETS gtsam2-shared LIBRARY DESTINATION lib )
endif(GTSAM2_BUILD_SHARED_LIBRARY)


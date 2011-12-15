# Build full gtsam library as a single library
# and also build tests 
set (gtsam_subdirs 
    base 
    geometry 
    inference 
    linear 
    nonlinear 
    slam)
 
set (ccolamd_srcs 
 3rdparty/CCOLAMD/Source/ccolamd.c 
 3rdparty/CCOLAMD/Source/ccolamd_global.c
 3rdparty/UFconfig/UFconfig.c) 

# install headers from 3rdparty libraries
add_subdirectory(3rdparty)

# Accumulate gtsam_srcs
set(gtsam_srcs ${ccolamd_srcs})

# Get all sources and headers from each 
foreach(subdir ${gtsam_subdirs})
    message(STATUS "Building ${subdir}")
    file(GLOB sub_gtsam_srcs "${subdir}/*.cpp")
    list(APPEND gtsam_srcs ${sub_gtsam_srcs})
    
    # install headers
    file(GLOB sub_gtsam_headers "${subdir}/*.h")
    install(FILES ${sub_gtsam_headers} DESTINATION include/gtsam/${subdir})
endforeach(subdir)

# build shared and static versions of the library
add_library(${PROJECT_NAME}-shared SHARED ${gtsam_srcs})
set_target_properties(${PROJECT_NAME}-shared PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
set_target_properties(${PROJECT_NAME}-shared PROPERTIES CLEAN_DIRECT_OUTPUT 1)
install(TARGETS ${PROJECT_NAME}-shared LIBRARY DESTINATION lib )

add_library(${PROJECT_NAME}-static STATIC ${gtsam_srcs})
set_target_properties(${PROJECT_NAME}-static PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
set_target_properties(${PROJECT_NAME}-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
install(TARGETS ${PROJECT_NAME}-static ARCHIVE DESTINATION lib)

# Get all sources and headers from each 
foreach(subdir ${gtsam_subdirs})
    # Build tests
    file(GLOB tests_srcs "${subdir}/tests/test*.cpp")
    foreach(test_src ${tests_srcs})
        get_filename_component(test_base ${test_src} NAME_WE)
        set( test_bin ${subdir}_${test_base} )
        add_executable(${test_bin} EXCLUDE_FROM_ALL ${test_src})
        add_dependencies(${test_bin} ${PROJECT_NAME}-static)
        add_dependencies(check ${test_bin})
        add_test(${subdir}/${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
        target_link_libraries(${test_bin} CppUnitLite gtsam-static)
        add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
    endforeach(test_src)
    
    # Build timing scripts
    file(GLOB time_srcs "${subdir}/tests/time*.cpp") 
    foreach(time_src ${time_srcs})
        get_filename_component(time_base ${time_src} NAME_WE)
        set( time_bin ${time_base} )
        add_executable(${time_bin} EXCLUDE_FROM_ALL ${time_src})
        add_dependencies(${test_bin} ${PROJECT_NAME}-static)
        add_dependencies(timing ${time_bin})
        target_link_libraries(${time_bin} CppUnitLite gtsam-static)
        add_custom_target(${time_base}.run ${EXECUTABLE_OUTPUT_PATH}${time_bin} ${ARGN})
    endforeach(time_src)
endforeach(subdir)


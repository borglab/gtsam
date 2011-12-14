# Build full gtsam library as a single library
# and also build tests 
set (gtsam_subdirs 
    base 
    geometry 
    inference 
    linear 
    nonlinear 
    slam)

# Include CCOLAMD source directly into gtsam
include_directories(
  3rdparty/UFconfig 
  3rdparty/CCOLAMD/Include
  ${CMAKE_SOURCE_DIR})
 
set (ccolamd_srcs 
 3rdparty/CCOLAMD/Source/ccolamd.c 
 3rdparty/CCOLAMD/Source/ccolamd_global.c
 3rdparty/UFconfig/UFconfig.c) 
 
set(gtsam_srcs ${ccolamd_srcs})
 
add_library(ccolamd STATIC ${ccolamd_srcs})

# Get all sources and headers from each 
foreach(subdir ${gtsam_subdirs})
    message(STATUS "Building ${subdir}")
    file(GLOB sub_gtsam_srcs "${subdir}/*.cpp")
    list(APPEND gtsam_srcs ${sub_gtsam_srcs})
    
    # Make a convenience library
    add_library(${subdir} STATIC ${sub_gtsam_srcs})
    list(APPEND inner_libs ${subdir})
    
    # Build tests
    file(GLOB tests_srcs "${subdir}/tests/test*.cpp")
    foreach(test_src ${tests_srcs})
        get_filename_component(test_base ${test_src} NAME_WE)
        set( test_bin ${EXECUTABLE_OUTPUT_PATH}/${subdir}/${test_base} )
        add_executable(${test_bin} ${test_src})
        #add_test(${subdir}/${test_base} ${subdir}/${test_base})
        #target_link_libraries(${subdir}/${test_base} ${inner_libs} CppUnitLite ccolamd)
        #add_custom_target(${test_base}.run ${EXECUTABLE_OUTPUT_PATH}/${test_base} ${ARGN})
    endforeach(test_src)
endforeach(subdir)

# build a single shared library
add_library(gtsam SHARED ${gtsam_srcs})


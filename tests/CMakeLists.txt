include_directories(
  3rdparty/UFconfig 
  3rdparty/CCOLAMD/Include
  ${CMAKE_SOURCE_DIR})

find_package(Boost COMPONENTS serialization REQUIRED)

# Build tests
file(GLOB tests_srcs "test*.cpp")
foreach(test_src ${tests_srcs})
    get_filename_component(test_base ${test_src} NAME_WE)
    set( test_bin ${test_base} )
    add_executable(${test_bin} EXCLUDE_FROM_ALL ${test_src})
    add_dependencies(check ${test_bin})
    add_dependencies(${test_bin} ${PROJECT_NAME}-static)
    add_test(${test_bin} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
    target_link_libraries(${test_bin} CppUnitLite gtsam-static ${Boost_SERIALIZATION_LIBRARY})
    add_custom_target(${test_base}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
endforeach(test_src)

# Build timing scripts
file(GLOB time_srcs "time*.cpp") 
foreach(time_src ${time_srcs})
    get_filename_component(time_base ${time_src} NAME_WE)
    set( time_bin ${time_base} )
    add_executable(${time_bin} EXCLUDE_FROM_ALL ${time_src})
    add_dependencies(timing ${time_bin})
    add_dependencies(${test_bin} ${PROJECT_NAME}-static)
    target_link_libraries(${time_bin} CppUnitLite gtsam-static)
    add_custom_target(${time_base}.run ${EXECUTABLE_OUTPUT_PATH}${time_bin} ${ARGN})
endforeach(time_src)
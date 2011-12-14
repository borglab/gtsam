# Build/install Wrap

# Build the executable itself
FILE(GLOB wrap_srcs "*.cpp")
LIST(REMOVE_ITEM wrap_srcs wrap.cpp)
add_library(wrapLib STATIC ${wrap_srcs})
add_executable(wrap wrap.cpp)
target_link_libraries(wrap wrapLib)

# Build tests
FILE(GLOB wrap_test_srcs "tests/test*.cpp")
add_definitions(-DTOPSRCDIR="${CMAKE_SOURCE_DIR}")
foreach(test_src ${wrap_test_srcs} )
    get_filename_component(test_base ${test_src} NAME_WE)
    set( test_bin wrap_${test_base} )
    add_executable(${test_bin} ${test_src})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}${test_bin})
    target_link_libraries(${test_bin} CppUnitLite gtsam wrapLib)
    add_custom_target(${test_bin}.run ${EXECUTABLE_OUTPUT_PATH}${test_bin} ${ARGN})
endforeach(test_src)




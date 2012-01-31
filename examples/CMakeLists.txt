add_custom_target(examples)

# Build example executables
FILE(GLOB example_srcs "*.cpp")
foreach(example_src ${example_srcs} )
    get_filename_component(example_base ${example_src} NAME_WE)
    set( example_bin ${example_base} )
    message(STATUS "Adding Example ${example_bin}")
    add_dependencies(examples ${example_bin})
    add_executable(${example_bin} ${example_src})
    target_link_libraries(${example_bin} gtsam-static)
endforeach(example_src)

add_subdirectory(vSLAMexample)

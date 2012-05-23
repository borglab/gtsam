add_custom_target(examples)

# Build example executables
FILE(GLOB example_srcs "*.cpp")
foreach(example_src ${example_srcs} )
    get_filename_component(example_base ${example_src} NAME_WE)
    set( example_bin ${example_base} )
    message(STATUS "Adding Example ${example_bin}")
    add_dependencies(examples ${example_bin})
    add_executable(${example_bin} ${example_src})
    
    # Disable building during make all/install
    if (GTSAM_ENABLE_INSTALL_EXAMPLE_FIX)
        set_target_properties(${example_bin} PROPERTIES EXCLUDE_FROM_ALL ON)
    endif()
    
    target_link_libraries(${example_bin} gtsam-static)
	if(NOT MSVC)
      add_custom_target(${example_bin}.run ${EXECUTABLE_OUTPUT_PATH}${example_bin} ${ARGN})
	endif()
endforeach(example_src)

add_subdirectory(vSLAMexample)

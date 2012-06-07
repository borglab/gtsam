if(NOT MSVC)
  add_custom_target(examples)
endif()

# Build example executables
FILE(GLOB example_srcs "*.cpp")
foreach(example_src ${example_srcs} )
    get_filename_component(example_base ${example_src} NAME_WE)
    set( example_bin ${example_base} )
    message(STATUS "Adding Example ${example_bin}")
	if(NOT MSVC)
      add_dependencies(examples ${example_bin})
	endif()
    add_executable(${example_bin} ${example_src})
    
    # Disable building during make all/install
    if (GTSAM_DISABLE_EXAMPLES_ON_INSTALL)
        set_target_properties(${example_bin} PROPERTIES EXCLUDE_FROM_ALL ON)
    endif()
    
    target_link_libraries(${example_bin} gtsam-static)
	if(NOT MSVC)
      add_custom_target(${example_bin}.run ${EXECUTABLE_OUTPUT_PATH}${example_bin} ${ARGN})
	endif()
	
	# Set up Visual Studio folder
	if(MSVC)
	  set_property(TARGET ${example_bin} PROPERTY FOLDER "Examples")
	endif()

endforeach(example_src)

# Install matlab components
if (GTSAM_BUILD_WRAP)
    if (GTSAM_INSTALL_MATLAB_TOOLBOX)
        # Examples
        if (GTSAM_INSTALL_MATLAB_EXAMPLES)
            message(STATUS "Installing Matlab Toolbox Examples")
            # Matlab files: *.m and *.fig
            file(GLOB matlab_examples_m "${GTSAM_SOURCE_ROOT_DIR}/examples/matlab/*.m")
            file(GLOB matlab_examples_fig "${GTSAM_SOURCE_ROOT_DIR}/examples/matlab/*.fig")
            set(matlab_examples ${matlab_examples_m} ${matlab_examples_fig})
            install(FILES ${matlab_examples} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam/examples)
            
            message(STATUS "Installing Matlab Toolbox Examples (Data)")
            # Data files: *.graph and *.txt
            file(GLOB matlab_examples_data_graph "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.graph")
            file(GLOB matlab_examples_data_txt "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.txt")
            set(matlab_examples_data ${matlab_examples_data_graph} ${matlab_examples_data_txt}) 
            install(FILES ${matlab_examples_data} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam/Data)
        endif (GTSAM_INSTALL_MATLAB_EXAMPLES)
    endif (GTSAM_INSTALL_MATLAB_TOOLBOX)
endif ()
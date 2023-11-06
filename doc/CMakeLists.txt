# Doxygen documentation configuration
option(GTSAM_BUILD_DOCS                  "Enable/Disable building of doxygen docs"   ON)

# configure doxygen
option(GTSAM_BUILD_DOC_HTML              "Enable/Disable doxygen HTML output"    ON)
option(GTSAM_BUILD_DOC_LATEX             "Enable/Disable doxygen LaTeX output"   OFF)

# add a target to generate API documentation with Doxygen
if (GTSAM_BUILD_DOCS)
    # Convert configuration to YES/NO variables
    if (GTSAM_BUILD_DOC_HTML)
        set(GTSAM_BUILD_DOC_HTML_YN "YES")
    else() 
        set(GTSAM_BUILD_DOC_HTML_YN "NO")
    endif()
    
    if (GTSAM_BUILD_DOC_LATEX)
        set(GTSAM_BUILD_DOC_LATEX_YN "YES")
    else() 
        set(GTSAM_BUILD_DOC_LATEX_YN "NO")
    endif()

    # GTSAM core subfolders
    set(gtsam_doc_subdirs 
        gtsam/base
        gtsam/basis 
        gtsam/discrete
        gtsam/geometry
        gtsam/hybrid
        gtsam/inference
        gtsam/linear
        gtsam/navigation
        gtsam/nonlinear
        gtsam/sam
        gtsam/sfm
        gtsam/slam
        gtsam/symbolic
        gtsam
    )
    
    # Optional GTSAM_UNSTABLE subfolders
    set(gtsam_unstable_doc_subdirs
        gtsam_unstable/base
        gtsam_unstable/discrete
        gtsam_unstable/dynamics
        gtsam_unstable/geometry
        gtsam_unstable/linear
        gtsam_unstable/nonlinear
        gtsam_unstable/partition
        gtsam_unstable/slam
        gtsam_unstable
    )
    
    # Build a list of folders to include depending on build options
    set(doc_subdirs ${gtsam_doc_subdirs})
    if (GTSAM_BUILD_UNSTABLE)
        list(APPEND doc_subdirs ${gtsam_unstable_doc_subdirs})
    endif()
    if (GTSAM_BUILD_EXAMPLES_ALWAYS)
        list(APPEND doc_subdirs examples)
    endif()
    
    # From subfolders, build a list with whitespace separation of paths
    set(GTSAM_DOXYGEN_INPUT_PATHS "")
    foreach(dir ${doc_subdirs})
        set(GTSAM_DOXYGEN_INPUT_PATHS "${GTSAM_DOXYGEN_INPUT_PATHS} ${PROJECT_SOURCE_DIR}/${dir}")
    endforeach()

    # Generate Doxyfile
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    
    # Add target to actually build documentation as configured
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
    
    # Clean target
    add_custom_target(doc_clean
        COMMAND
            cmake -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/latex
        COMMAND
            cmake -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/html
        COMMENT "Removing Doxygen documentation"
    )
endif()

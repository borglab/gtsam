# Build the tools...

# Where to find the *.usage files for the --help option.
include_directories (${PROJECT_BINARY_DIR}/man)
# Only needed if target_compile_definitions is not supported
add_definitions (${PROJECT_DEFINITIONS})

# Loop over all the tools, specifying the source and library.
add_custom_target (tools ALL)
foreach (TOOL ${TOOLS})

  add_executable (${TOOL} ${TOOL}.cpp)
  if (MAINTAINER)
    add_dependencies (${TOOL} usage)
  endif ()
  add_dependencies (tools ${TOOL})

  set_source_files_properties (${TOOL}.cpp PROPERTIES
    OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/man/${TOOL}.usage)

  target_link_libraries (${TOOL} ${PROJECT_LIBRARIES} ${HIGHPREC_LIBRARIES})

endforeach ()

if (MSVC OR CMAKE_CONFIGURATION_TYPES)
  # Add _d suffix for your debug versions of the tools
  set_target_properties (${TOOLS} PROPERTIES
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
endif ()

if (APPLE AND NOT CMAKE_VERSION VERSION_LESS 2.8.12)
  # Ensure that the package is relocatable
  set_target_properties (${TOOLS} PROPERTIES
    INSTALL_RPATH "@loader_path/../lib${LIB_SUFFIX}")
endif ()

# Specify where the tools are installed, adding them to the export targets
install (TARGETS ${TOOLS} EXPORT targets DESTINATION bin)

# Put all the tools into a folder in the IDE
set_property (TARGET tools ${TOOLS} PROPERTY FOLDER tools)

# Create the scripts for downloading the data files on non-Windows
# systems.  This needs to substitute ${GEOGRAPHICLIB_DATA} as the
# default data directory.  These are installed under sbin, because it is
# expected to be run with write access to /usr/local.
if (NOT WIN32)
  foreach (SCRIPT ${SCRIPTS})
    configure_file (${SCRIPT}.sh scripts/${SCRIPT} @ONLY)
    add_custom_command (OUTPUT ${SCRIPT}
      COMMAND ${CMAKE_COMMAND} -E
        copy scripts/${SCRIPT} ${SCRIPT} && chmod +x ${SCRIPT}
      DEPENDS ${SCRIPT}.sh)
    install (PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${SCRIPT} DESTINATION sbin)
  endforeach ()
  add_custom_target (scripts ALL DEPENDS ${SCRIPTS})
endif ()

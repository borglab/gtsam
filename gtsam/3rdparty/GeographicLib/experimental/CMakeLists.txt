# Build experimental code

# Only needed if target_compile_definitions is not supported
add_definitions (${PROJECT_DEFINITIONS})

add_custom_target (experimental)

set (EXPTOOLS JacobiConformal)

foreach (EXPTOOL ${EXPTOOLS})

  set (EXPSOURCE ${EXPTOOL}.cpp)
  if (EXPTOOL STREQUAL "JacobiConformal")
    set (EXPSOURCE ${EXPSOURCE} JacobiConformal.hpp)
  endif ()
  add_executable (${EXPTOOL} EXCLUDE_FROM_ALL ${EXPSOURCE})
  add_dependencies (experimental ${EXPTOOL})
  target_link_libraries (${EXPTOOL} ${PROJECT_LIBRARIES} ${HIGHPREC_LIBRARIES})

endforeach ()

if (MSVC OR CMAKE_CONFIGURATION_TYPES)
  # Add _d suffix for your debug versions of the tools
  set_target_properties (${EXPTOOLS} PROPERTIES
    DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
endif ()

if (APPLE AND RELATIVE_LIBDIR)
  # Ensure that the package is relocatable
  set_target_properties (${EXPTOOLS} PROPERTIES
    INSTALL_RPATH "@loader_path/${RELATIVE_LIBDIR}")
endif ()

# Put all the tools into a folder in the IDE
set_property (TARGET experimental ${TOOLS} PROPERTY FOLDER experimental)

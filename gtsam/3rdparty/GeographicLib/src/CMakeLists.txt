# Build the library...

# Include all the .cpp files in the library.
file (GLOB SOURCES [A-Za-z]*.cpp)
file (GLOB HEADERS
  ${PROJECT_BINARY_DIR}/include/GeographicLib/Config.h
  ../include/GeographicLib/[A-Za-z]*.hpp)

# Define the library and specify whether it is shared or not.
if (GEOGRAPHICLIB_SHARED_LIB)
  add_library (${PROJECT_SHARED_LIBRARIES} SHARED ${SOURCES} ${HEADERS})
endif ()
if (GEOGRAPHICLIB_STATIC_LIB)
  add_library (${PROJECT_STATIC_LIBRARIES} STATIC ${SOURCES} ${HEADERS})
endif ()

# Set the version number on the library
if (MSVC)
  if (GEOGRAPHICLIB_SHARED_LIB)
    set_target_properties (${PROJECT_SHARED_LIBRARIES} PROPERTIES
      VERSION "${LIBVERSION_BUILD}" OUTPUT_NAME ${LIBNAME}
      IMPORT_SUFFIX -i.lib)
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
      set_target_properties (${PROJECT_SHARED_LIBRARIES} PROPERTIES
        COMPILE_DEFINITIONS GEOGRAPHICLIB_SHARED_LIB=1)
    else ()
      target_compile_definitions (${PROJECT_SHARED_LIBRARIES}
        PUBLIC GEOGRAPHICLIB_SHARED_LIB=1)
    endif ()
  endif ()
  if (GEOGRAPHICLIB_STATIC_LIB)
    set_target_properties (${PROJECT_STATIC_LIBRARIES} PROPERTIES
      VERSION "${LIBVERSION_BUILD}" OUTPUT_NAME ${LIBNAME})
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
      set_target_properties (${PROJECT_STATIC_LIBRARIES} PROPERTIES
        COMPILE_DEFINITIONS GEOGRAPHICLIB_SHARED_LIB=0)
    else ()
      target_compile_definitions (${PROJECT_STATIC_LIBRARIES}
        PUBLIC GEOGRAPHICLIB_SHARED_LIB=0)
    endif ()
  endif ()
else ()
  set_target_properties (
    ${PROJECT_SHARED_LIBRARIES} ${PROJECT_STATIC_LIBRARIES} PROPERTIES
    VERSION "${LIBVERSION_BUILD}" SOVERSION "${LIBVERSION_API}"
    OUTPUT_NAME ${LIBNAME})
  if (APPLE AND GEOGRAPHICLIB_PRECISION EQUAL 5)
    if (GEOGRAPHICLIB_SHARED_LIB)
      target_link_libraries (${PROJECT_SHARED_LIBRARIES} ${HIGHPREC_LIBRARIES})
    endif ()
    if (GEOGRAPHICLIB_STATIC_LIB)
      target_link_libraries (${PROJECT_STATIC_LIBRARIES} ${HIGHPREC_LIBRARIES})
    endif ()
  endif ()
endif ()

if (NOT CMAKE_VERSION VERSION_LESS 2.8.11)
  if (GEOGRAPHICLIB_SHARED_LIB)
    target_include_directories (${PROJECT_SHARED_LIBRARIES} INTERFACE
      $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
  endif ()
  if (GEOGRAPHICLIB_STATIC_LIB)
    target_include_directories (${PROJECT_STATIC_LIBRARIES} INTERFACE
      $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
  endif ()
endif ()

# Specify where the library is installed, adding it to the export targets
install (TARGETS ${PROJECT_SHARED_LIBRARIES} ${PROJECT_STATIC_LIBRARIES}
  EXPORT targets
  # A potentially useful option.  However it's only supported in recent
  # versions of cmake (2.8.12 and later?).  So comment out for now.
  # INCLUDES DESTINATION include
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib${LIB_SUFFIX}
  ARCHIVE DESTINATION lib${LIB_SUFFIX})

if (MSVC AND PACKAGE_DEBUG_LIBS)
  if (GEOGRAPHICLIB_SHARED_LIB)
    install (FILES
      "${PROJECT_BINARY_DIR}/lib/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}-i.lib"
      DESTINATION lib${LIB_SUFFIX} CONFIGURATIONS Release)
    install (PROGRAMS
      "${PROJECT_BINARY_DIR}/bin/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}.dll"
      DESTINATION bin CONFIGURATIONS Release)
  endif ()
  if (GEOGRAPHICLIB_STATIC_LIB)
    install (FILES
      "${PROJECT_BINARY_DIR}/lib/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}.lib"
      DESTINATION lib${LIB_SUFFIX} CONFIGURATIONS Release)
  endif ()
endif ()

if (MSVC AND GEOGRAPHICLIB_SHARED_LIB)
  if (NOT CMAKE_VERSION VERSION_LESS 3.1)
    install (FILES $<TARGET_PDB_FILE:${PROJECT_SHARED_LIBRARIES}>
      DESTINATION bin OPTIONAL)
  else ()
    if (NOT CMAKE_VERSION VERSION_LESS 3.0)
      # Suppress 3.0 warnings about use of the LOCATION target property
      cmake_policy (SET CMP0026 OLD)
    endif ()
    # Install pdb file for shared library.
    foreach (_c ${CMAKE_CONFIGURATION_TYPES})
      string (TOUPPER ${_c} _C)
      get_target_property (_P ${PROJECT_SHARED_LIBRARIES} LOCATION_${_C})
      get_filename_component (_D ${_P} PATH)
      get_filename_component (_N ${_P} NAME_WE)
      set (_PDB ${_D}/${_N}.pdb)
      install (FILES ${_PDB} DESTINATION bin CONFIGURATIONS ${_c} OPTIONAL)
    endforeach ()
  endif ()
endif ()

# Put the library into a folder in the IDE
set_target_properties (
  ${PROJECT_SHARED_LIBRARIES} ${PROJECT_STATIC_LIBRARIES}
  PROPERTIES FOLDER library)

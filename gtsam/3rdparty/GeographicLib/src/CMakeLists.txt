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
      VERSION "${LIBVERSIONFULL}" OUTPUT_NAME ${LIBNAME} IMPORT_SUFFIX -i.lib
      PDB_NAME ${LIBNAME}${CMAKE_DEBUG_POSTFIX})
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
      VERSION "${LIBVERSIONFULL}" OUTPUT_NAME ${LIBNAME})
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
    VERSION "${LIBVERSIONFULL}" SOVERSION "${LIBVERSION}"
    OUTPUT_NAME ${LIBNAME})
endif ()

# Specify where the library is installed, adding it to the export depends
install (TARGETS ${PROJECT_SHARED_LIBRARIES} ${PROJECT_STATIC_LIBRARIES}
  EXPORT depends
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib${LIB_SUFFIX}
  ARCHIVE DESTINATION lib${LIB_SUFFIX})

if (MSVC AND PACKAGE_DEBUG_LIBS)
  if (GEOGRAPHICLIB_SHARED_LIB)
    install (FILES
      "${CMAKE_CURRENT_BINARY_DIR}/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}-i.lib"
      DESTINATION lib${LIB_SUFFIX} CONFIGURATIONS Release)
    install (PROGRAMS
      "${CMAKE_CURRENT_BINARY_DIR}/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}.dll"
      DESTINATION bin CONFIGURATIONS Release)
  endif ()
  if (GEOGRAPHICLIB_STATIC_LIB)
    install (FILES
      "${CMAKE_CURRENT_BINARY_DIR}/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}.lib"
      DESTINATION lib${LIB_SUFFIX} CONFIGURATIONS Release)
  endif ()
endif ()

if (MSVC AND GEOGRAPHICLIB_SHARED_LIB)
  # Install pdb file for shared library in debug mode.
  get_target_property (_P ${PROJECT_SHARED_LIBRARIES} LOCATION_DEBUG)
  get_filename_component (_P ${_P} PATH)
  get_target_property (_N ${PROJECT_SHARED_LIBRARIES} PDB_NAME)
  set (_PDB ${_P}/${_N}.pdb)
  install (FILES ${_PDB} DESTINATION bin
    RENAME ${LIBNAME}${CMAKE_DEBUG_POSTFIX}.pdb CONFIGURATIONS Debug)
endif ()

# Put the library into a folder in the IDE
set_target_properties (
  ${PROJECT_SHARED_LIBRARIES} ${PROJECT_STATIC_LIBRARIES}
  PROPERTIES FOLDER library)

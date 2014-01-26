# Compile a bunch of tiny example programs.  These are built with the
# "exampleprograms" target.  These are mainly for including as examples
# within the doxygen documentation; however, compiling them catches some
# obvious blunders.
file (GLOB EXAMPLE_SOURCES example-*.cpp)
set (EXAMPLES)
add_definitions (${PROJECT_DEFINITIONS})

foreach (EXAMPLE_SOURCE ${EXAMPLE_SOURCES})
  get_filename_component (EXAMPLE ${EXAMPLE_SOURCE} NAME_WE)
  set (EXAMPLES ${EXAMPLES} ${EXAMPLE})
  add_executable (${EXAMPLE} EXCLUDE_FROM_ALL ${EXAMPLE_SOURCE})
  target_link_libraries (${EXAMPLE} ${PROJECT_LIBRARIES})
endforeach ()

set (OTHER_EXAMPLES GeoidToGTX.cpp)

foreach (EXAMPLE_SOURCE ${OTHER_EXAMPLES})
  get_filename_component (EXAMPLE ${EXAMPLE_SOURCE} NAME_WE)
  set (EXAMPLES ${EXAMPLES} ${EXAMPLE})
  add_executable (${EXAMPLE} EXCLUDE_FROM_ALL ${EXAMPLE_SOURCE})
  target_link_libraries (${EXAMPLE} ${PROJECT_LIBRARIES})
endforeach ()

find_package (OpenMP QUIET)

if (OPENMP_FOUND)
  set_target_properties (GeoidToGTX PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS} COMPILE_DEFINITIONS HAVE_OPENMP=1)
  if (NOT WIN32)
    set_target_properties (GeoidToGTX PROPERTIES LINK_FLAGS ${OpenMP_CXX_FLAGS})
  endif ()
  message (STATUS "Example program GeoidToGTX will use OpenMP")
else ()
  message (STATUS "Example program GeoidToGTX will not use OpenMP")
endif ()

add_custom_target (exampleprograms DEPENDS ${EXAMPLES})

if (MSVC)
  get_target_property (_LIBTYPE ${PROJECT_LIBRARIES} TYPE)
  if (_LIBTYPE STREQUAL "SHARED_LIBRARY")
    # Copy the shared library on Windows systems to this directory
    # (examples) so that the tests can be run.
    add_custom_command (TARGET exampleprograms POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E
      copy $<TARGET_FILE:${PROJECT_LIBRARIES}> ${CMAKE_CFG_INTDIR}
      COMMENT "Copying shared library to examples directory")
  endif ()
endif ()

# Put all the examples into a folder in the IDE
set_property (TARGET exampleprograms ${EXAMPLES} PROPERTY FOLDER examples)

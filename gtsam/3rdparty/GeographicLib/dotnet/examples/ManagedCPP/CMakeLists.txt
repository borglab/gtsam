# Compile (but don't install) a bunch of tiny example programs for
# NETGeographic.  These are mainly for including as examples within the
# doxygen documentation; however, compiling them catches most obvious
# blunders.

file (GLOB EXAMPLE_SOURCES example-*.cpp)
set (EXAMPLES)

foreach (EXAMPLE_SOURCE ${EXAMPLE_SOURCES})
  get_filename_component (EXAMPLE ${EXAMPLE_SOURCE} NAME_WE)
  set (EXAMPLE "net${EXAMPLE}")
  set (EXAMPLES ${EXAMPLES} ${EXAMPLE})
  add_executable (${EXAMPLE} EXCLUDE_FROM_ALL ${EXAMPLE_SOURCE})
  add_dependencies (${EXAMPLE} ${NETGEOGRAPHICLIB_LIBRARIES})
  set_target_properties (${EXAMPLE} PROPERTIES COMPILE_FLAGS "/clr")
  # This is set up for Release builds only.  Change
  # Release/NETGeographic.dll to Debug/NETGeographic_d.dll for Debug
  # builds.
  set_target_properties (${EXAMPLE} PROPERTIES VS_DOTNET_REFERENCES
    "${CMAKE_CURRENT_BINARY_DIR}/../../NETGeographicLib/Release/NETGeographic.dll")
endforeach ()

string (REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
string (REPLACE "/EHsc" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

add_custom_target (netexamples DEPENDS ${EXAMPLES})

get_target_property (_LIBTYPE ${PROJECT_LIBRARIES} TYPE)
if (_LIBTYPE STREQUAL "SHARED_LIBRARY")
  # Copy the shared library on Windows systems to this directory
  # (examples) so that the tests can be run.
  add_custom_command (TARGET netexamples POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E
    copy $<TARGET_FILE:${PROJECT_LIBRARIES}> ${CMAKE_CFG_INTDIR}
    COMMENT "Copying shared library to examples directory")
endif ()

# Put all the examples into a folder in the IDE
set_property (TARGET netexamples ${EXAMPLES} PROPERTY FOLDER .NET-examples)

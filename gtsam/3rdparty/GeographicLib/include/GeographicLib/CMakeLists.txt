# Install the header files including the generated Config.h (which is in
# the build tree).
file (GLOB HEADERS [A-Za-z]*.hpp)
install (FILES ${HEADERS} DESTINATION include/GeographicLib)
install (FILES ${CMAKE_CURRENT_BINARY_DIR}/Config.h
  DESTINATION include/GeographicLib)

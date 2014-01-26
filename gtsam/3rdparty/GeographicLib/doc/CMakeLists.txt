# Where the html versions of the man pages (extension .1.html) are
# found.
set (MANDIR ${PROJECT_BINARY_DIR}/man)

# Build up a list of the .1.html files.
set (HTMLMAN)
foreach (TOOL ${TOOLS})
  set (HTMLMAN ${HTMLMAN} ${MANDIR}/${TOOL}.1.html)
endforeach ()

if (COMMON_INSTALL_PATH)
  set (INSTALL_DOC_DIR "share/doc/GeographicLib")
else ()
  set (INSTALL_DOC_DIR "doc")
endif ()

# Run doxygen, if available

# First assemble a list of all the files the documentation uses.  Add a
# dependency on htmlman (from man/CMakeLists.txt).  Use html/index.html
# as the make target.  To make this target, copy the non-doxygen
# generated files into html/.  Run doxfile.in thru cmake's config
# process so that absolute path names are used and so that the pathnames
# are properly stripped by doxygen (via STRIP_FROM_PATH). The
# distrib-doc target copies the html directory into the source tree.

# If doxygen is not available, only the install step (from the source
# tree) is done.

file (MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html-stage)
if (DOXYGEN_FOUND)
  configure_file (doxyfile.in doxyfile)
  configure_file (doxyfile-c.in doxyfile-c)
  configure_file (doxyfile-for.in doxyfile-for)
  configure_file (doxyfile-net.in doxyfile-net)
  file (GLOB SOURCES
    ../src/[A-Za-z]*.cpp ../include/GeographicLib/[A-Za-z]*.hpp
    ../tools/[A-Za-z]*.cpp ../examples/[A-Za-z]*.cpp
    ../legacy/C/*.[ch] ../legacy/Fortran/*.for ../legacy/Fortran/*.inc
    ../dotnet/NETGeographicLib/*.cpp ../dotnet/NETGeographicLib/*.h
    ../dotnet/examples/CS/*.cs ../dotnet/examples/ManagedCPP/*.cpp
    ../dotnet/examples/VB/*.vb)
  file (GLOB EXTRA_FILES ../maxima/[A-Za-z]*.mac
    tmseries30.html geodseries30.html ../LICENSE.txt)
  file (GLOB FIGURES *.png)
  file (COPY ${EXTRA_FILES} DESTINATION html-stage)
  add_custom_target (doc ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/index.html)
  add_dependencies (doc htmlman)
  add_custom_command (OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/html/index.html
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doxyfile
      ${CMAKE_CURRENT_BINARY_DIR}/doxyfile-c
      ${CMAKE_CURRENT_BINARY_DIR}/doxyfile-for
      ${CMAKE_CURRENT_BINARY_DIR}/doxyfile-net
      GeographicLib.dox geodesic-c.dox geodesic-for.dox NETGeographicLib.dox
      ${SOURCES} ${EXTRA_FILES} ${FIGURES} ${HTMLMAN}
    COMMAND ${CMAKE_COMMAND} -E remove_directory html
    COMMAND ${CMAKE_COMMAND} -E copy_directory html-stage html
    COMMAND ${DOXYGEN_EXECUTABLE} doxyfile > doxygen.log
    COMMAND ${DOXYGEN_EXECUTABLE} doxyfile-c > doxygen-c.log
    COMMAND ${DOXYGEN_EXECUTABLE} doxyfile-for > doxygen-for.log
    COMMAND ${DOXYGEN_EXECUTABLE} doxyfile-net > doxygen-net.log
    COMMENT "Generating html documentation tree")
  install (DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
    DESTINATION ${INSTALL_DOC_DIR})
else ()
  file (COPY ../LICENSE.txt DESTINATION html)
  # Need to absolute path on destination to support old versions of cmake
  configure_file (index.html.in html/index.html)
  configure_file (utilities.html.in html/utilities.html)
  install (FILES
    ${CMAKE_CURRENT_BINARY_DIR}/html/LICENSE.txt
    ${CMAKE_CURRENT_BINARY_DIR}/html/index.html
    ${CMAKE_CURRENT_BINARY_DIR}/html/utilities.html
    DESTINATION ${INSTALL_DOC_DIR}/html)
endif ()

# Finally install the JavaScript files
file (GLOB SCRIPTDRIVERS scripts/[A-Za-z]*.html)
file (GLOB JSSCRIPTS scripts/GeographicLib/[A-Za-z]*.js)
install (FILES ${SCRIPTDRIVERS} DESTINATION ${INSTALL_DOC_DIR}/scripts)
install (FILES ${JSSCRIPTS}
  DESTINATION ${INSTALL_DOC_DIR}/scripts/GeographicLib)

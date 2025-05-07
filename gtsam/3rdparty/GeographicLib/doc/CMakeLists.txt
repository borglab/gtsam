# Where the html versions of the man pages (extension .1.html) are
# found.
# Where to find the *.usage files for the --help option.
if (RELEASE)
  set (MANDIR ${PROJECT_SOURCE_DIR}/man)
else ()
  set (MANDIR ${PROJECT_BINARY_DIR}/man)
endif ()

# Build up a list of the .1.html files.
set (HTMLMAN)
foreach (TOOL ${TOOLS})
  set (HTMLMAN ${HTMLMAN} ${MANDIR}/${TOOL}.1.html)
endforeach ()

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
if (BUILD_DOCUMENTATION)

  configure_file (GeographicLib.dox.in GeographicLib.dox @ONLY)
  configure_file (doxyfile.in doxyfile @ONLY)
  file (GLOB CXXSOURCES
    ../src/[A-Za-z]*.cpp ../include/GeographicLib/[A-Za-z]*.hpp
    ../tools/[A-Za-z]*.cpp ../examples/[A-Za-z]*.cpp
    ../experimental/[A-Za-z]*.[hc]pp)
  file (GLOB EXTRA_FILES ../maxima/[A-Za-z]*.mac
    tmseries30.html geodseries30.html ../LICENSE.txt)
  file (GLOB FIGURES *.png *.svg *.gif)
  file (COPY ${EXTRA_FILES} DESTINATION html-stage)
  if (DOCDIR)
    add_custom_target (doc ALL
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/index.html)
  else ()
    add_custom_target (doc
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/index.html)
  endif ()
  if (NOT RELEASE)
    add_dependencies (doc htmlman)
  endif ()
  if (CMAKE_VERSION VERSION_LESS 3.17.0)
    set (RMDIR remove_directory)
  else ()
    set (RMDIR rm -rf)
  endif ()
  add_custom_command (OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/html/index.html
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doxyfile
      ${CMAKE_CURRENT_BINARY_DIR}/GeographicLib.dox
      ${CXXSOURCES} ${EXTRA_FILES} ${FIGURES} ${HTMLMAN}
    COMMAND ${CMAKE_COMMAND} -E copy ${HTMLMAN} html-stage
    COMMAND ${CMAKE_COMMAND} -E ${RMDIR} html
    COMMAND ${CMAKE_COMMAND} -E copy_directory html-stage html
    COMMAND ${DOXYGEN_EXECUTABLE} doxyfile > doxygen.log
    COMMENT "Generating C++ documentation tree")

else ()

  file (COPY ../LICENSE.txt DESTINATION html)
  configure_file (index.html.in html/index.html)
  configure_file (utilities.html.in html/utilities.html)

endif ()

if (DOCDIR)
  install (DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
    DESTINATION ${DOCDIR})
endif ()

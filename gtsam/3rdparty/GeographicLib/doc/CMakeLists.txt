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
  set (DOCTARGETS)

  configure_file (GeographicLib.dox.in GeographicLib.dox @ONLY)
  configure_file (doxyfile.in doxyfile @ONLY)
  file (GLOB CXXSOURCES
    ../src/[A-Za-z]*.cpp ../include/GeographicLib/[A-Za-z]*.hpp
    ../tools/[A-Za-z]*.cpp ../examples/[A-Za-z]*.cpp
    ../examples/[A-Za-z]*.hpp)
  file (GLOB EXTRA_FILES ../maxima/[A-Za-z]*.mac
    tmseries30.html geodseries30.html ../LICENSE.txt)
  file (GLOB FIGURES *.png *.svg *.gif)
  file (COPY ${EXTRA_FILES} DESTINATION html-stage)
  add_custom_target (cxxdoc ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/index.html)
  add_dependencies (cxxdoc htmlman)
  add_custom_command (OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/html/index.html
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doxyfile
      ${CMAKE_CURRENT_BINARY_DIR}/GeographicLib.dox
      ${CXXSOURCES} ${EXTRA_FILES} ${FIGURES} ${HTMLMAN}
    COMMAND ${CMAKE_COMMAND} -E remove_directory html
    COMMAND ${CMAKE_COMMAND} -E copy_directory html-stage html
    COMMAND ${DOXYGEN_EXECUTABLE} doxyfile > doxygen.log
    COMMENT "Generating C++ documentation tree")
  set (DOCTARGETS ${DOCTARGETS} cxxdoc)

  configure_file (doxyfile-c.in doxyfile-c @ONLY)
  file (GLOB CSOURCES ../legacy/C/*.[ch])
  add_custom_target (cdoc ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/C/index.html)
  add_dependencies (cdoc cxxdoc)
  add_custom_command (OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/html/C/index.html
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doxyfile-c geodesic-c.dox ${CSOURCES}
    COMMAND ${DOXYGEN_EXECUTABLE} doxyfile-c > doxygen-c.log
    COMMENT "Generating C documentation tree")
  set (DOCTARGETS ${DOCTARGETS} cdoc)

  configure_file (doxyfile-for.in doxyfile-for @ONLY)
  file (GLOB FORTRANSOURCES ../legacy/Fortran/*.for ../legacy/Fortran/*.inc)
  add_custom_target (fortrandoc ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/Fortran/index.html)
  add_dependencies (fortrandoc cxxdoc)
  add_custom_command (OUTPUT
      ${CMAKE_CURRENT_BINARY_DIR}/html/Fortran/index.html
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doxyfile-for
      geodesic-for.dox ${FORTRANSOURCES}
    COMMAND ${DOXYGEN_EXECUTABLE} doxyfile-for > doxygen-for.log
    COMMENT "Generating Fortran documentation tree")
  set (DOCTARGETS ${DOCTARGETS} fortrandoc)

  configure_file (doxyfile-net.in doxyfile-net @ONLY)
  file (GLOB DOTNETSOURCES
    ../dotnet/NETGeographicLib/*.cpp ../dotnet/NETGeographicLib/*.h
    ../dotnet/examples/CS/*.cs ../dotnet/examples/ManagedCPP/*.cpp
    ../dotnet/examples/VB/*.vb)
  add_custom_target (dotnetdoc ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/NET/index.html)
  add_dependencies (dotnetdoc cxxdoc)
  add_custom_command (OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/html/NET/index.html
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doxyfile-net
      NETGeographicLib.dox ${DOTNETSOURCES}
    COMMAND ${DOXYGEN_EXECUTABLE} doxyfile-net > doxygen-net.log
    COMMENT "Generating .NET documentation tree")
  set (DOCTARGETS ${DOCTARGETS} dotnetdoc)

  if (JSDOC)
    file (GLOB JSSOURCES
      ../js/src/*.js ../js/GeographicLib.md  ../js/doc/*.md)
    add_custom_target (jsdoc ALL
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/js/index.html)
    add_dependencies (jsdoc cxxdoc)
    add_custom_command (OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/html/js/index.html
      DEPENDS ${JSSOURCES}
      COMMAND ${JSDOC} --verbose -d html/js
        -u ${PROJECT_SOURCE_DIR}/js/doc
        -c ${PROJECT_SOURCE_DIR}/js/conf.json
        -R ${PROJECT_SOURCE_DIR}/js/GeographicLib.md
        ${PROJECT_SOURCE_DIR}/js/src > jsdoc.log
      COMMENT "Generating JavaScript documentation tree")
    set (DOCTARGETS ${DOCTARGETS} jsdoc)
  endif ()

  if (SPHINX)
    file (GLOB PYTHONSOURCES
      ../python/geographiclib/*.py ../python/doc/*.rst ../python/doc/conf.py)
    add_custom_target (pythondoc ALL
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/html/python/index.html)
    add_dependencies (pythondoc cxxdoc)
    add_custom_command (OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/html/python/index.html
      DEPENDS ${PYTHONSOURCES}
      COMMAND ${SPHINX} -v -b html -d python-doctree
        ${PROJECT_SOURCE_DIR}/python/doc html/python > pythondoc.log
      COMMENT "Generating python documentation tree")
    set (DOCTARGETS ${DOCTARGETS} pythondoc)
  endif ()

  add_custom_target (doc ALL)
  add_dependencies (doc ${DOCTARGETS})

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

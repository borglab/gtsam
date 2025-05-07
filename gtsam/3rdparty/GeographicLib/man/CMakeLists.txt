if (RELEASE)

  # In RELEASE mode the man pages are already available in man, html,
  # and usage formats This cmake file then just has to install the man
  # pages.  The html pages will be copied by doc/CMakeLists.txt and the
  # usage pages will be included by tools/CMakeLists.txt
  if (MANDIR AND BINDIR)
    foreach (TOOL ${TOOLS})
      install (FILES ${TOOL}.1 DESTINATION ${MANDIR}/man1)
    endforeach ()
  endif ()

else ()

  include (../cmake/maintainer-man.cmake)

endif ()

if (NOT WIN32)
  # Install the man pages for the scripts.  This is the same for RELEASE
  # and non-RELEASE
  set (SYSMANPAGES)
  foreach (SCRIPT ${SCRIPTS})
    string (REPLACE geographiclib-get- "" DATA ${SCRIPT})
    set (SYSMANPAGES ${SYSMANPAGES} ${CMAKE_CURRENT_BINARY_DIR}/${SCRIPT}.8)
    configure_file (script.8.in ${SCRIPT}.8 @ONLY)
  endforeach ()
  if (MANDIR AND SBINDIR)
    install (FILES ${SYSMANPAGES} DESTINATION ${MANDIR}/man8)
  endif ()
endif ()

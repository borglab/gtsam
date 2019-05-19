# The man pages are maintained as .pod (plain old documentatoin) files.
# In maintainer mode, there are used to create real man pages (extension
# .1), usage files (extension .usage) for including in the tool itself,
# and html versions of the man pages (extension .1.html) for use from
# the doxygen generated documentation

# Only the maintainer tries to generate the derived files and the .usage
# files are in the build tree.  For non-maintainers, the .usages files
# are in the source tree.
if (MAINTAINER)
  add_custom_target (distrib-man)
  add_custom_target (man ALL)
endif ()

set (MANPAGES)
set (USAGE)
set (HTMLMAN)
set (SYSMANPAGES)

# Loop over the tools building up lists of the derived files.  Also in
# maintainer mode, specify how the derived files are created.  The sed
# replacements for the .1.html files glue in a style sheet and implement
# cross-referencing between the tools.  The .usage files are generated
# by a shell script makeusage.sh.
foreach (TOOL ${TOOLS})
  set (MANPAGES ${MANPAGES} ${CMAKE_CURRENT_BINARY_DIR}/${TOOL}.1)
  set (USAGE ${USAGE} ${CMAKE_CURRENT_BINARY_DIR}/${TOOL}.usage)
  set (HTMLMAN ${HTMLMAN} ${CMAKE_CURRENT_BINARY_DIR}/${TOOL}.1.html)
  if (MAINTAINER)
    add_custom_command (OUTPUT ${TOOL}.1
      COMMAND pod2man --center=\"GeographicLib Utilities\"
        --release=\"GeographicLib ${PROJECT_VERSION}\"
        ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL}.pod > ${TOOL}.1
      COMMENT "Building man page for ${TOOL}"
      MAIN_DEPENDENCY ${TOOL}.pod)
    add_custom_command (OUTPUT ${TOOL}.1.html COMMAND
        pod2html --title "'${TOOL}(1)'"
          --noindex ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL}.pod |
        sed
          -e 's%<head>%<head><link href="http://search.cpan.org/s/style.css" rel="stylesheet" type="text/css">%'
          -e 's%<code>\\\([^<>]*\\\)\(\\\(.\\\)\)</code>%<a href="\\1.\\2.html">&</a>%'g > ${TOOL}.1.html &&
        cp ${TOOL}.1.html ../doc/html-stage/
      COMMENT "Building html version of man page for ${TOOL}"
      MAIN_DEPENDENCY ${TOOL}.pod)
    add_custom_command (OUTPUT ${TOOL}.usage
      COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/makeusage.sh
        ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL}.pod ${PROJECT_VERSION}
        > ${TOOL}.usage
      COMMENT "Building usage code for ${TOOL}"
      MAIN_DEPENDENCY ${TOOL}.pod)
  else ()
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL}.usage)
      file (COPY ${TOOL}.usage DESTINATION .)
    else ()
      configure_file (dummy.usage.in ${TOOL}.usage @ONLY)
    endif ()
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL}.1)
      file (COPY ${TOOL}.1 DESTINATION .)
    else ()
      configure_file (dummy.1.in ${TOOL}.1 @ONLY)
    endif ()
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL}.1.html)
      file (COPY ${TOOL}.1.html DESTINATION .)
    else ()
      configure_file (dummy.1.html.in ${TOOL}.1.html @ONLY)
    endif ()
    if (DOXYGEN_FOUND)
      file (COPY ${CMAKE_CURRENT_BINARY_DIR}/${TOOL}.1.html
        DESTINATION ../doc/html-stage)
    endif ()
  endif ()
endforeach ()

if (NOT WIN32)
  foreach (SCRIPT ${SCRIPTS})
    string (REPLACE geographiclib-get- "" DATA ${SCRIPT})
    set (SYSMANPAGES ${SYSMANPAGES} ${CMAKE_CURRENT_BINARY_DIR}/${SCRIPT}.8)
    configure_file (script.8.in ${SCRIPT}.8 @ONLY)
  endforeach ()
endif ()

# Add the extra maintainer tasks into the dependency list.  The
# distrib-man target copies the derived documentation files into the
# source tree.
if (MAINTAINER)
  add_custom_target (manpages ALL DEPENDS ${MANPAGES}
    COMMENT "Building all the man pages")
  add_custom_target (usage ALL DEPENDS ${USAGE}
    COMMENT "Converting the man pages to online usage")
  add_custom_target (htmlman ALL DEPENDS ${HTMLMAN}
    COMMENT "Building html versions of the man pages")
  add_dependencies (man manpages usage htmlman)
  add_dependencies (distrib-man man)
  add_custom_command (TARGET distrib-man
    COMMAND
      for f in ${MANPAGES} ${USAGE} ${HTMLMAN}\; do
        install -C -m 644 "$$f" ${CMAKE_CURRENT_SOURCE_DIR}\; done
    COMMENT "Installing man documentation page in source tree")
else ()
  add_custom_target (htmlman ALL DEPENDS ${HTMLMAN})
endif ()

# Install the man pages.
install (FILES ${MANPAGES} DESTINATION share/man/man1)
if (NOT WIN32)
  install (FILES ${SYSMANPAGES} DESTINATION share/man/man8)
endif ()

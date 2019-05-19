# Install matlab files.
if (COMMON_INSTALL_PATH)
  # More Octave friendly would be "share/octave/site/m"
  set (INSTALL_MATLAB_DIR "share/matlab")
else ()
  set (INSTALL_MATLAB_DIR "matlab")
endif ()
file (GLOB MATLAB_FILES geographiclib/[A-Za-z]*.m)
install (FILES ${MATLAB_FILES} DESTINATION ${INSTALL_MATLAB_DIR}/geographiclib)
# Install "private" functions
file (GLOB PRIVATE_MATLAB_FILES geographiclib/private/[A-Za-z]*.m)
install (FILES ${PRIVATE_MATLAB_FILES}
  DESTINATION ${INSTALL_MATLAB_DIR}/geographiclib/private)
# Install "legacy" functions
file (GLOB LEGACY_FILES geographiclib-legacy/[A-Za-z]*.m)
install (FILES ${LEGACY_FILES}
  DESTINATION ${INSTALL_MATLAB_DIR}/geographiclib-legacy)

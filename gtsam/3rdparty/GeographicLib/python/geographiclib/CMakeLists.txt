# Install the python files.
# Probably full-time python users should install the python package from
# http://pypi.python.org/pypi/geographiclib
file (GLOB PYTHON_FILES [A-Za-z_]*.py)
file (GLOB TEST_FILES test/[A-Za-z_]*.py)
if (COMMON_INSTALL_PATH)
  set (INSTALL_PYTHON_DIR "lib${LIB_SUFFIX}/python/site-packages")
else ()
  set (INSTALL_PYTHON_DIR "python")
endif ()

install (FILES ${PYTHON_FILES}
  DESTINATION ${INSTALL_PYTHON_DIR}/geographiclib)
install (FILES ${TEST_FILES}
  DESTINATION ${INSTALL_PYTHON_DIR}/geographiclib/test)

# Only install setup.py under Windows because the installation tree is,
# e.g., c:/Program Files/GeographicLib/${INSTALL_PYTHON_PATH}.  Don't
# install on other systems where this command would create, e.g.,
# /usr/local/${INSTALL_PYTHON_PATH}/setup.py which is likely to cause
# conflicts.
if (NOT COMMON_INSTALL_PATH)
  install (FILES ../setup.py DESTINATION ${INSTALL_PYTHON_DIR})
endif ()

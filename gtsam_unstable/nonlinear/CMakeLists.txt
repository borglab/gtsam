# Install headers
file(GLOB nonlinear_headers "*.h")
install(FILES ${nonlinear_headers} DESTINATION include/gtsam_unstable/nonlinear)

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

# Add all tests
add_subdirectory(tests)

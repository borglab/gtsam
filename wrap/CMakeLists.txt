# Build/install Wrap

set(WRAP_BOOST_LIBRARIES
  Boost::system
  Boost::filesystem
  Boost::thread
)

# Allow for disabling serialization to handle errors related to Clang's linker
option(GTSAM_WRAP_SERIALIZATION "If enabled, allows for wrapped objects to be saved via boost.serialization" ON)

# Build the executable itself
file(GLOB wrap_srcs "*.cpp")
file(GLOB wrap_headers "*.h")
list(REMOVE_ITEM wrap_srcs ${CMAKE_CURRENT_SOURCE_DIR}/wrap.cpp)
add_library(wrap_lib STATIC ${wrap_srcs} ${wrap_headers})
target_include_directories(wrap_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)
if (NOT GTSAM_WRAP_SERIALIZATION)
  target_compile_definitions(wrap_lib PUBLIC -DWRAP_DISABLE_SERIALIZE)
endif()

# Apply build flags:
gtsam_apply_build_flags(wrap_lib)

target_link_libraries(wrap_lib ${WRAP_BOOST_LIBRARIES})
gtsam_assign_source_folders(${wrap_srcs} ${wrap_headers})
add_executable(wrap wrap.cpp)
target_link_libraries(wrap PRIVATE wrap_lib)

# Set folder in Visual Studio
file(RELATIVE_PATH relative_path "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(wrap_lib wrap PROPERTIES FOLDER "${relative_path}")

# Install wrap binary and export target
install(TARGETS wrap EXPORT GTSAM-exports DESTINATION ${CMAKE_INSTALL_BINDIR})
list(APPEND GTSAM_EXPORTED_TARGETS wrap)
set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)

# Install matlab header
install(FILES matlab.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/wrap)

# Build tests
add_subdirectory(tests)

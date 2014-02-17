# Build/install Wrap

set(WRAP_BOOST_LIBRARIES ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

# Allow for disabling serialization to handle errors related to Clang's linker
option(GTSAM_WRAP_SERIALIZATION "If enabled, allows for wrapped objects to be saved via boost.serialization" ON)
if (NOT GTSAM_WRAP_SERIALIZATION)
    add_definitions(-DWRAP_DISABLE_SERIALIZE)
endif()  

# Build the executable itself
file(GLOB wrap_srcs "*.cpp")
file(GLOB wrap_headers "*.h")
list(REMOVE_ITEM wrap_srcs ${CMAKE_CURRENT_SOURCE_DIR}/wrap.cpp)
add_library(wrap_lib STATIC ${wrap_srcs} ${wrap_headers})
target_link_libraries(wrap_lib ${WRAP_BOOST_LIBRARIES})
gtsam_assign_source_folders(${wrap_srcs} ${wrap_headers})
add_executable(wrap wrap.cpp)
target_link_libraries(wrap wrap_lib ${WRAP_BOOST_LIBRARIES})

# Set folder in Visual Studio
file(RELATIVE_PATH relative_path "${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(wrap_lib wrap PROPERTIES FOLDER "${relative_path}")

# Install wrap binary and export target
install(TARGETS wrap EXPORT GTSAM-exports DESTINATION bin)
list(APPEND GTSAM_EXPORTED_TARGETS wrap)
set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)

# Install matlab header
install(FILES matlab.h DESTINATION include/wrap)

# Build tests
add_subdirectory(tests)


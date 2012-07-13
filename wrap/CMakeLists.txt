# Build/install Wrap

set(WRAP_BOOST_LIBRARIES ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

# Build the executable itself
file(GLOB wrap_srcs "*.cpp")
file(GLOB wrap_headers "*.h")
list(REMOVE_ITEM wrap_srcs ${CMAKE_CURRENT_SOURCE_DIR}/wrap.cpp)
add_library(wrap_lib STATIC ${wrap_srcs} ${wrap_headers})
gtsam_assign_source_folders(${wrap_srcs} ${wrap_headers})
add_executable(wrap wrap.cpp)
target_link_libraries(wrap wrap_lib ${WRAP_BOOST_LIBRARIES})

# Install wrap binary and export target
if (GTSAM_INSTALL_WRAP)
    install(TARGETS wrap EXPORT GTSAM-exports DESTINATION bin)
    list(APPEND GTSAM_EXPORTED_TARGETS wrap)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
endif(GTSAM_INSTALL_WRAP)

# Install matlab header
install(FILES matlab.h DESTINATION include/wrap)

# Build tests
if (GTSAM_BUILD_TESTS)
    set(wrap_local_libs wrap_lib ${WRAP_BOOST_LIBRARIES}) 
    gtsam_add_subdir_tests("wrap" "${wrap_local_libs}" "${wrap_local_libs}" "") 
endif(GTSAM_BUILD_TESTS)



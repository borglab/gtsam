# Build/install Wrap

find_package(Boost 1.42 COMPONENTS system filesystem thread REQUIRED)

# Build the executable itself
file(GLOB wrap_srcs "*.cpp")
list(REMOVE_ITEM wrap_srcs ${CMAKE_CURRENT_SOURCE_DIR}/wrap.cpp)
add_library(wrap_lib STATIC ${wrap_srcs})
add_executable(wrap wrap.cpp)
target_link_libraries(wrap wrap_lib ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

# Install wrap binary
if (GTSAM_INSTALL_WRAP)
    install(TARGETS wrap DESTINATION bin)
endif(GTSAM_INSTALL_WRAP)

# Install matlab header
install(FILES matlab.h DESTINATION include/wrap)

# Build tests
if (GTSAM_BUILD_TESTS) 
    set(wrap_local_libs wrap_lib ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}) 
    gtsam_add_subdir_tests("wrap" "${wrap_local_libs}" "${wrap_local_libs}" "") 
endif(GTSAM_BUILD_TESTS)



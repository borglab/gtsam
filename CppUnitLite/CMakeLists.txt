# Build/install CppUnitLite

file(GLOB cppunitlite_headers "*.h")
file(GLOB cppunitlite_src "*.cpp")

add_library(CppUnitLite STATIC ${cppunitlite_src})

option(GTSAM_INSTALL_CPPUNITLITE "Enable/Disable installation of CppUnitLite library" ON)
if (GTSAM_INSTALL_CPPUNITLITE)
    install(FILES ${cppunitlite_headers} DESTINATION include/CppUnitLite)
    install(TARGETS CppUnitLite ARCHIVE DESTINATION lib)
endif(GTSAM_INSTALL_CPPUNITLITE)

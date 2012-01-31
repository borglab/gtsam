# Build/install CppUnitLite

FILE(GLOB cppunitlite_headers "*.h")
FILE(GLOB cppunitlite_src "*.cpp")

ADD_LIBRARY(CppUnitLite STATIC ${cppunitlite_src})

option(GTSAM_INSTALL_CPPUNITLITE "Enable/Disable installation of CppUnitLite library" ON)
if (GTSAM_INSTALL_CPPUNITLITE)
    install(FILES ${cppunitlite_headers} DESTINATION include/CppUnitLite)
    install(TARGETS CppUnitLite ARCHIVE DESTINATION lib)
endif(GTSAM_INSTALL_CPPUNITLITE)

# Build/install CppUnitLite

FILE(GLOB cppunitlite_headers "*.h")
FILE(GLOB cppunitlite_src "*.cpp")

ADD_LIBRARY(CppUnitLite STATIC ${cppunitlite_src})

install(FILES ${cppunitlite_headers} DESTINATION include/CppUnitLite)
install(TARGETS CppUnitLite ARCHIVE DESTINATION lib)

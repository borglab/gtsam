CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

SET(lib_target "CppUnitLite")
FILE(GLOB cppunitlite_src "./*.cpp")

############################# COMPILE LIBRARY
ADD_LIBRARY(${lib_target} STATIC ${cppunitlite_src})
#TARGET_LINK_LIBRARIES(${lib_target})

############################# INSTALL

#message(STATUS "install ${lib_target} to ${CMAKE_SOURCE_DIR}/lib") 
#install(TARGETS ${lib_target}
#	    ARCHIVE DESTINATION lib COMPONENT dev
#	    LIBRARY DESTINATION lib COMPONENT dev)

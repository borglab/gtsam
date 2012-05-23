# Build vSLAMexample

message(STATUS "Adding Example vISAMexample")
add_executable(vISAMexample vISAMexample.cpp vSLAMutils.cpp)
target_link_libraries(vISAMexample gtsam-static)
if(NOT MSVC)
  add_dependencies(examples vISAMexample)
else()
  set_property(TARGET vISAMexample PROPERTY FOLDER "Examples")
endif()

message(STATUS "Adding Example vSFMexample")
add_executable(vSFMexample vSFMexample.cpp vSLAMutils.cpp)
target_link_libraries(vSFMexample gtsam-static)
if(NOT MSVC)
  add_dependencies(examples vSFMexample)
else()
  set_property(TARGET vSFMexample PROPERTY FOLDER "Examples")
endif()




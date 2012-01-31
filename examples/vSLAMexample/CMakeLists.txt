# Build vSLAMexample

message(STATUS "Adding Example vISAMexample")
add_executable(vISAMexample vISAMexample.cpp vSLAMutils.cpp)
target_link_libraries(vISAMexample gtsam-static)
add_dependencies(examples vISAMexample)

message(STATUS "Adding Example vSFMexample")
add_executable(vSFMexample vSFMexample.cpp vSLAMutils.cpp)
target_link_libraries(vSFMexample gtsam-static)
add_dependencies(examples vSFMexample)




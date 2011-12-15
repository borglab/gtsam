# Build vSLAMexample

set ( srcs 
  Feature2D.cpp
  vSLAMutils.cpp
)
add_library(vSLAMexample ${srcs})

add_executable(vISAMexample vISAMexample.cpp)
target_link_libraries(vISAMexample vSLAMexample gtsam)

add_executable(vSFMexample vSFMexample.cpp)
target_link_libraries(vSFMexample vSLAMexample gtsam)





include_directories("${PROJECT_SOURCE_DIR}/gtsam")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

#include_directories(${EIGEN_INCLUDE_DIRS})

file(GLOB base_src "base/*.cpp")
file(GLOB geometry_src "geometry/*.cpp")
file(GLOB inference_src "inference/*.cpp")
file(GLOB linear_src "linear/*.cpp")
file(GLOB nonlinear_src "nonlinear/*.cpp")
file(GLOB slam_src "slam/*.cpp")
file(GLOB symbolic_src "symbolic/*.cpp")

#wrap_python("base" ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME} ${base_src})
wrap_python("geometry" ${PROJECT_SOURCE_DIR}/python/gtsam ${geometry_src})

#add_python_export_library(${PROJECT_NAME}_test ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME}
#  ${AUTOGEN_TEST_FILES}
#)
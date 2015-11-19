# Obtain Dependencies
# Boost Python
find_package(Boost COMPONENTS python REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Find Python
find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Numpy_Eigen
include_directories(${CMAKE_SOURCE_DIR}/gtsam/3rdparty/numpy_eigen/include/)

add_subdirectory(handwritten)
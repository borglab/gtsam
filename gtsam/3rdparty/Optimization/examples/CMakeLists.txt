# Find Eigen library
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/")
find_package(Eigen3 3.3.3 REQUIRED)
if(EIGEN3_FOUND)
message(STATUS "Found Eigen3 library (version ${EIGEN3_VERSION})")
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}\n")
else()
message(STATUS "Eigen library not found!")
endif()

# Add the Eigen include directories
include_directories(${EIGEN3_INCLUDE_DIR})

# Enable faster instruction sets (SIMD/AVX)
set(ENABLE_VECTORIZATION OFF CACHE BOOL "Enable vectorized instruction sets (SIMD/AVX)? [disabled by default]")
message(STATUS "Processing CMakeLists for header-only library ${PROJECT_NAME}:\n")

if(${ENABLE_VECTORIZATION})
message(STATUS "Enabling SIMD/AVX instruction sets")
add_definitions(-march=native)
endif()


# Enable OpenMP (if available)
set(ENABLE_OPENMP ON CACHE BOOL "Enable OpenMP (if available)")

if(${ENABLE_OPENMP})
find_package(OpenMP)
	if(OPENMP_FOUND)
	message(STATUS "Found OpenMP! Turning on support for parallelization\n")

	# Append OpenMP compilation and linker flags to all executables
	SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")
	endif()
endif()

# EXAMPLE EXECUTABLES

# Build a minimal working example to demonstrate truncated-Newton trust-region optimization over the sphere S2
add_executable(Riemannian_optimization_example ${CMAKE_CURRENT_SOURCE_DIR}/Riemannian_optimization_example.cpp)
target_link_libraries(Riemannian_optimization_example Optimization)

# Build a minimal working example to demonstrate the use of the simplified Euclidean truncated-Newton trust-region interface to minimize the McCormick function
add_executable(Euclidean_optimization_example ${CMAKE_CURRENT_SOURCE_DIR}/Euclidean_optimization_example.cpp)
target_link_libraries(Euclidean_optimization_example Optimization)

# Build a minimal working example to demonstrate the proximal gradient method on a group-sparse LASSO problem
add_executable(proximal_gradient_example ${CMAKE_CURRENT_SOURCE_DIR}/proximal_gradient_example.cpp)
target_link_libraries(proximal_gradient_example Optimization)

# Build a minimal working example to demonstrate the use of the ADMM algorithm
add_executable(ADMM_example ${CMAKE_CURRENT_SOURCE_DIR}/ADMM_example.cpp)
target_link_libraries(ADMM_example Optimization)

# Build a minimal working example to demonstrate the LOBPCG method
add_executable(LOBPCG_example ${CMAKE_CURRENT_SOURCE_DIR}/LOBPCG_example.cpp)
target_link_libraries(LOBPCG_example Optimization)







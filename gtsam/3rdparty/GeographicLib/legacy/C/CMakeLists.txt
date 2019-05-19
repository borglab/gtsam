project (GeographicLib-C C)

cmake_minimum_required (VERSION 2.8.4)

# Set a default build type for single-configuration cmake generators if
# no build type is set.
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif ()

# Make the compiler more picky.
if (MSVC)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W4")
else ()
  set (CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -Wall -Wextra -Wfloat-conversion -Wno-array-bounds -pedantic")
endif ()

include (CheckCSourceCompiles)
if (MSVC)
  set (CMAKE_REQUIRED_FLAGS "${CMAKE_C_FLAGS} /WX")
else ()
  set (CMAKE_REQUIRED_LIBRARIES m)
  set (CMAKE_REQUIRED_FLAGS "${CMAKE_C_FLAGS} -Werror")
endif ()
# Check whether the C99 math function: hypot, atanh, etc. are available.
check_c_source_compiles (
  "#include <math.h>
int main() {
  int q;
  return (int)(hypot(3.0, 4.0) + atanh(0.8) + cbrt(8.0) +
               remquo(100.0, 90.0, &q) +
               remainder(100.0, 90.0) + copysign(1.0, -0.0));
}\n" C99_MATH)
if (C99_MATH)
  add_definitions (-DHAVE_C99_MATH=1)
else ()
  add_definitions (-DHAVE_C99_MATH=0)
endif ()

if (CONVERT_WARNINGS_TO_ERRORS)
  if (MSVC)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /WX")
  else ()
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")
  endif ()
endif ()

set (TOOLS direct inverse planimeter geodtest)

foreach (TOOL ${TOOLS})
  add_executable (${TOOL} ${TOOL}.c geodesic.c geodesic.h)
  if (NOT MSVC)
    target_link_libraries (${TOOL} m)
  endif ()
endforeach ()

# Turn on testing
enable_testing ()

# Run the test suite
add_test (NAME geodtest COMMAND geodtest)

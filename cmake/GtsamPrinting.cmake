# Based on https://github.com/jimbraun/XCDF/blob/master/cmake/CMakePadString.cmake
function(string_pad RESULT_NAME DESIRED_LENGTH VALUE)
    string(LENGTH "${VALUE}" VALUE_LENGTH)
    math(EXPR REQUIRED_PADS "${DESIRED_LENGTH} - ${VALUE_LENGTH}")
    set(PAD ${VALUE})
    if(REQUIRED_PADS GREATER 0)
        math(EXPR REQUIRED_MINUS_ONE "${REQUIRED_PADS} - 1")
        foreach(FOO RANGE ${REQUIRED_MINUS_ONE})
            set(PAD "${PAD} ")
        endforeach()
    endif()
    set(${RESULT_NAME} "${PAD}" PARENT_SCOPE)
endfunction()

set(GTSAM_PRINT_SUMMARY_PADDING_LENGTH 50 CACHE STRING "Padding of cmake summary report lines after configuring.")
mark_as_advanced(GTSAM_PRINT_SUMMARY_PADDING_LENGTH)

# print configuration variables with automatic padding
# Usage:
#   print_config(${GTSAM_BUILD_TESTS} "Build Tests")
function(print_config config msg)
  string_pad(padded_config ${GTSAM_PRINT_SUMMARY_PADDING_LENGTH} " ${config}")
  message(STATUS "${padded_config}: ${msg}")
endfunction()

# print configuration variable with enabled/disabled value
# Usage:
#   print_enabled_config(${GTSAM_BUILD_TESTS} "Build Tests                ")
function(print_enabled_config config msg)
    string_pad(padded_msg ${GTSAM_PRINT_SUMMARY_PADDING_LENGTH} " ${msg}")
    if (config)
        message(STATUS "${padded_msg}: Enabled")
    else ()
        message(STATUS "${padded_msg}: Disabled")
    endif ()
endfunction()


# Print "  var: ${var}" padding with spaces as needed
function(print_padded variable_name)
  string_pad(padded_prop ${GTSAM_PRINT_SUMMARY_PADDING_LENGTH} " ${variable_name}")
  message(STATUS "${padded_prop}: ${${variable_name}}")
endfunction()


# Prints all the relevant CMake build options for a given target:
function(print_build_options_for_target target_name_)
  print_padded(GTSAM_COMPILE_FEATURES_PUBLIC)
  # print_padded(GTSAM_COMPILE_OPTIONS_PRIVATE)
  print_padded(GTSAM_COMPILE_OPTIONS_PUBLIC)
  # print_padded(GTSAM_COMPILE_DEFINITIONS_PRIVATE)
  print_padded(GTSAM_COMPILE_DEFINITIONS_PUBLIC)

  foreach(build_type ${GTSAM_CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${build_type}" build_type_toupper)
    # print_padded(GTSAM_COMPILE_OPTIONS_PRIVATE_${build_type_toupper})
    print_padded(GTSAM_COMPILE_OPTIONS_PUBLIC_${build_type_toupper})
    # print_padded(GTSAM_COMPILE_DEFINITIONS_PRIVATE_${build_type_toupper})
    print_padded(GTSAM_COMPILE_DEFINITIONS_PUBLIC_${build_type_toupper})
  endforeach()
endfunction()

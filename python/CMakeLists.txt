set(PROJECT_PYTHON_SOURCE_DIR ${PROJECT_SOURCE_DIR}/python)
set(GTSAM_PYTHON_BUILD_DIRECTORY ${PROJECT_BINARY_DIR}/python)

if (NOT GTSAM_BUILD_PYTHON)
    return()
endif()

# Install development dependencies to build wrapper
message(STATUS "Installing Python development dependencies")
execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-m" "pip" "install" "-r" "dev_requirements.txt"
                WORKING_DIRECTORY ${PROJECT_PYTHON_SOURCE_DIR}
                OUTPUT_QUIET)

# Generate setup.py.
file(READ "${PROJECT_SOURCE_DIR}/README.md" README_CONTENTS)
configure_file(${PROJECT_PYTHON_SOURCE_DIR}/setup.py.in
                ${GTSAM_PYTHON_BUILD_DIRECTORY}/setup.py)

# Supply MANIFEST.in for older versions of Python
file(COPY ${PROJECT_PYTHON_SOURCE_DIR}/MANIFEST.in
     DESTINATION ${GTSAM_PYTHON_BUILD_DIRECTORY})

set(WRAP_BUILD_TYPE_POSTFIXES ${GTSAM_BUILD_TYPE_POSTFIXES})

include(PybindWrap)

############################################################
## Load the necessary files to compile the wrapper

# Load the pybind11 code


# This is required to avoid an error in modern pybind11 cmake scripts:
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Prefer system pybind11 first, if not found, rely on bundled version:
find_package(pybind11 CONFIG QUIET)
if (NOT pybind11_FOUND)
        add_subdirectory(${PROJECT_SOURCE_DIR}/wrap/pybind11 pybind11)
endif()

# Set the wrapping script variable
set(PYBIND_WRAP_SCRIPT "${PROJECT_SOURCE_DIR}/wrap/scripts/pybind_wrap.py")
############################################################

add_custom_target(gtsam_header DEPENDS "${PROJECT_SOURCE_DIR}/gtsam/gtsam.i")
add_custom_target(gtsam_unstable_header DEPENDS "${PROJECT_SOURCE_DIR}/gtsam_unstable/gtsam_unstable.i")

# ignoring the non-concrete types (type aliases)
set(ignore
    gtsam::Point2
    gtsam::Point3
    gtsam::ISAM2ThresholdMapValue
    gtsam::FactorIndices
    gtsam::FactorIndexSet
    gtsam::IndexPairSet
    gtsam::IndexPairSetMap
    gtsam::IndexPairVector
    gtsam::BetweenFactorPose2s
    gtsam::BetweenFactorPose3s
    gtsam::FixedLagSmootherKeyTimestampMap
    gtsam::FixedLagSmootherKeyTimestampMapValue
    gtsam::Point2Vector
    gtsam::Point2Pairs
    gtsam::Point3Pairs
    gtsam::Pose3Pairs
    gtsam::Pose3Vector
    gtsam::Rot3Vector
    gtsam::KeyVector
    gtsam::BinaryMeasurementsPoint3
    gtsam::BinaryMeasurementsUnit3
    gtsam::BinaryMeasurementsRot3
    gtsam::DiscreteKey
    gtsam::KeyPairDoubleMap
    gtsam::gtsfm::MatchIndicesMap
    gtsam::gtsfm::KeypointsVector
    gtsam::gtsfm::SfmTrack2dVector)

set(interface_headers
    ${PROJECT_SOURCE_DIR}/gtsam/gtsam.i
    ${PROJECT_SOURCE_DIR}/gtsam/base/base.i
    ${PROJECT_SOURCE_DIR}/gtsam/inference/inference.i
    ${PROJECT_SOURCE_DIR}/gtsam/discrete/discrete.i
    ${PROJECT_SOURCE_DIR}/gtsam/geometry/geometry.i
    ${PROJECT_SOURCE_DIR}/gtsam/linear/linear.i
    ${PROJECT_SOURCE_DIR}/gtsam/nonlinear/nonlinear.i
    ${PROJECT_SOURCE_DIR}/gtsam/nonlinear/values.i
    ${PROJECT_SOURCE_DIR}/gtsam/nonlinear/custom.i
    ${PROJECT_SOURCE_DIR}/gtsam/symbolic/symbolic.i
    ${PROJECT_SOURCE_DIR}/gtsam/sam/sam.i
    ${PROJECT_SOURCE_DIR}/gtsam/slam/slam.i
    ${PROJECT_SOURCE_DIR}/gtsam/sfm/sfm.i
    ${PROJECT_SOURCE_DIR}/gtsam/navigation/navigation.i
    ${PROJECT_SOURCE_DIR}/gtsam/basis/basis.i
    ${PROJECT_SOURCE_DIR}/gtsam/hybrid/hybrid.i
)

set(GTSAM_PYTHON_TARGET gtsam_py)
set(GTSAM_PYTHON_UNSTABLE_TARGET gtsam_unstable_py)

pybind_wrap(${GTSAM_PYTHON_TARGET} # target
            "${interface_headers}" # interface_headers
            "gtsam.cpp" # generated_cpp
            "gtsam" # module_name
            "gtsam" # top_namespace
            "${ignore}" # ignore_classes
            ${PROJECT_PYTHON_SOURCE_DIR}/gtsam/gtsam.tpl
            gtsam # libs
            "gtsam;gtsam_header" # dependencies
            ${GTSAM_ENABLE_BOOST_SERIALIZATION} # use_boost_serialization
            )

set_target_properties(${GTSAM_PYTHON_TARGET} PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    OUTPUT_NAME "gtsam"
    LIBRARY_OUTPUT_DIRECTORY "${GTSAM_PYTHON_BUILD_DIRECTORY}/gtsam"
    DEBUG_POSTFIX "" # Otherwise you will have a wrong name
    RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
    )

# Set the path for the GTSAM python module
set(GTSAM_MODULE_PATH ${GTSAM_PYTHON_BUILD_DIRECTORY}/gtsam)

# Copy all python files to build folder.
copy_directory("${CMAKE_CURRENT_SOURCE_DIR}/gtsam"
        "${GTSAM_MODULE_PATH}")

# Hack to get python test and util files copied every time they are modified
file(GLOB GTSAM_PYTHON_TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/tests/*.py")
foreach(test_file ${GTSAM_PYTHON_TEST_FILES})
        get_filename_component(test_file_name ${test_file} NAME)
        configure_file(${test_file} "${GTSAM_MODULE_PATH}/tests/${test_file_name}" COPYONLY)
endforeach()
file(GLOB GTSAM_PYTHON_UTIL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/utils/*.py")
foreach(util_file ${GTSAM_PYTHON_UTIL_FILES})
        configure_file(${util_file} "${GTSAM_MODULE_PATH}/utils/${test_file}" COPYONLY)
endforeach()
file(GLOB GTSAM_PYTHON_PREAMBLE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/preamble/*.h")
foreach(util_file ${GTSAM_PYTHON_PREAMBLE_FILES})
        configure_file(${util_file} "${GTSAM_MODULE_PATH}/preamble/${test_file}" COPYONLY)
endforeach()
file(GLOB GTSAM_PYTHON_SPECIALIZATION_FILES "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/specializations/*.h")
foreach(util_file ${GTSAM_PYTHON_SPECIALIZATION_FILES})
        configure_file(${util_file} "${GTSAM_MODULE_PATH}/specializations/${test_file}" COPYONLY)
endforeach()

# Common directory for data/datasets stored with the package.
# This will store the data in the Python site package directly.
file(COPY "${GTSAM_SOURCE_DIR}/examples/Data" DESTINATION "${GTSAM_MODULE_PATH}")

# Add gtsam as a dependency to the install target
set(GTSAM_PYTHON_DEPENDENCIES ${GTSAM_PYTHON_TARGET})


if(GTSAM_UNSTABLE_BUILD_PYTHON)
    set(ignore
            gtsam::Point2
            gtsam::Point3
            gtsam::ISAM2ThresholdMapValue
            gtsam::FactorIndices
            gtsam::FactorIndexSet
            gtsam::BetweenFactorPose3s
            gtsam::Point2Vector
            gtsam::Pose3Vector
            gtsam::KeyVector
            gtsam::BinaryMeasurementsPoint3
            gtsam::BinaryMeasurementsUnit3
            gtsam::BinaryMeasurementsRot3
            gtsam::CameraSetCal3_S2
            gtsam::CameraSetCal3Bundler
            gtsam::CameraSetCal3Unified
            gtsam::CameraSetCal3Fisheye
            gtsam::KeyPairDoubleMap
            gtsam::gtsfm::MatchIndicesMap
            gtsam::gtsfm::KeypointsVector
            gtsam::gtsfm::SfmTrack2dVector)


    pybind_wrap(${GTSAM_PYTHON_UNSTABLE_TARGET} # target
            ${PROJECT_SOURCE_DIR}/gtsam_unstable/gtsam_unstable.i # interface_header
            "gtsam_unstable.cpp" # generated_cpp
            "gtsam_unstable" # module_name
            "gtsam" # top_namespace
            "${ignore}" # ignore_classes
            ${PROJECT_PYTHON_SOURCE_DIR}/gtsam_unstable/gtsam_unstable.tpl
            gtsam_unstable # libs
            "gtsam_unstable;gtsam_unstable_header" # dependencies
            ${GTSAM_ENABLE_BOOST_SERIALIZATION} # use_boost_serialization
            )

    set_target_properties(${GTSAM_PYTHON_UNSTABLE_TARGET} PROPERTIES
            INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
            INSTALL_RPATH_USE_LINK_PATH TRUE
            OUTPUT_NAME "gtsam_unstable"
            LIBRARY_OUTPUT_DIRECTORY "${GTSAM_PYTHON_BUILD_DIRECTORY}/gtsam_unstable"
            DEBUG_POSTFIX "" # Otherwise you will have a wrong name
            RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
            )

    set(GTSAM_UNSTABLE_MODULE_PATH ${GTSAM_PYTHON_BUILD_DIRECTORY}/gtsam_unstable)

    # Copy all python files to build folder.
    copy_directory("${CMAKE_CURRENT_SOURCE_DIR}/gtsam_unstable"
            "${GTSAM_UNSTABLE_MODULE_PATH}")

    # Hack to get python test files copied every time they are modified
    file(GLOB GTSAM_UNSTABLE_PYTHON_TEST_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/gtsam_unstable/" "${CMAKE_CURRENT_SOURCE_DIR}/gtsam_unstable/tests/*.py")
    foreach(test_file ${GTSAM_UNSTABLE_PYTHON_TEST_FILES})
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/gtsam_unstable/${test_file}" "${GTSAM_UNSTABLE_MODULE_PATH}/${test_file}" COPYONLY)
    endforeach()

    # Add gtsam_unstable to the install target
    list(APPEND GTSAM_PYTHON_DEPENDENCIES ${GTSAM_PYTHON_UNSTABLE_TARGET})

endif()

# Add custom target so we can install with `make python-install`
set(GTSAM_PYTHON_INSTALL_TARGET python-install)
add_custom_target(${GTSAM_PYTHON_INSTALL_TARGET}
        COMMAND ${PYTHON_EXECUTABLE} -m pip install .
        DEPENDS ${GTSAM_PYTHON_DEPENDENCIES}
        WORKING_DIRECTORY ${GTSAM_PYTHON_BUILD_DIRECTORY})

# Custom make command to run all GTSAM Python tests
add_custom_target(
        python-test
        COMMAND
          ${CMAKE_COMMAND} -E env # add package to python path so no need to install
          "PYTHONPATH=${GTSAM_PYTHON_BUILD_DIRECTORY}/$ENV{PYTHONPATH}"
          ${PYTHON_EXECUTABLE} -m unittest discover -v -s .
          DEPENDS ${GTSAM_PYTHON_DEPENDENCIES} ${GTSAM_PYTHON_TEST_FILES}
          WORKING_DIRECTORY "${GTSAM_PYTHON_BUILD_DIRECTORY}/gtsam/tests")

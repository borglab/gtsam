set(GTSAM_PYTHON_BUILD_DIRECTORY ${PROJECT_BINARY_DIR}/python)

if (NOT GTSAM_BUILD_PYTHON)
    return()
endif()

# Common directory for storing data/datasets stored with the package.
# This will store the data in the Python site package directly.
set(GTSAM_PYTHON_DATASET_DIR "./gtsam/Data")

# Generate setup.py.
file(READ "${PROJECT_SOURCE_DIR}/README.md" README_CONTENTS)
configure_file(${PROJECT_SOURCE_DIR}/python/setup.py.in
                ${GTSAM_PYTHON_BUILD_DIRECTORY}/setup.py)

set(WRAP_USE_CUSTOM_PYTHON_LIBRARY ${GTSAM_USE_CUSTOM_PYTHON_LIBRARY})
set(WRAP_PYTHON_VERSION ${GTSAM_PYTHON_VERSION})

include(PybindWrap)

add_custom_target(gtsam_header DEPENDS "${PROJECT_SOURCE_DIR}/gtsam/gtsam.i")
add_custom_target(gtsam_unstable_header DEPENDS "${PROJECT_SOURCE_DIR}/gtsam_unstable/gtsam_unstable.i")

# ignoring the non-concrete types (type aliases)
set(ignore
    gtsam::Point2
    gtsam::Point3
    gtsam::LieVector
    gtsam::LieMatrix
    gtsam::ISAM2ThresholdMapValue
    gtsam::FactorIndices
    gtsam::FactorIndexSet
    gtsam::IndexPairSetMap
    gtsam::IndexPairVector
    gtsam::BetweenFactorPose2s
    gtsam::BetweenFactorPose3s
    gtsam::Point2Vector
    gtsam::Pose3Vector
    gtsam::KeyVector
    gtsam::BinaryMeasurementsUnit3
    gtsam::KeyPairDoubleMap)

pybind_wrap(gtsam_py # target
            ${PROJECT_SOURCE_DIR}/gtsam/gtsam.i # interface_header
            "gtsam.cpp" # generated_cpp
            "gtsam" # module_name
            "gtsam" # top_namespace
            "${ignore}" # ignore_classes
            ${PROJECT_SOURCE_DIR}/python/gtsam/gtsam.tpl
            gtsam # libs
            "gtsam;gtsam_header" # dependencies
            ON # use_boost
            )

set_target_properties(gtsam_py PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    OUTPUT_NAME "gtsam"
    LIBRARY_OUTPUT_DIRECTORY "${GTSAM_PYTHON_BUILD_DIRECTORY}/gtsam"
    DEBUG_POSTFIX "" # Otherwise you will have a wrong name
    RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
    )

set(GTSAM_MODULE_PATH ${GTSAM_PYTHON_BUILD_DIRECTORY}/gtsam)

# Symlink all tests .py files to build folder.
create_symlinks("${CMAKE_CURRENT_SOURCE_DIR}/gtsam"
        "${GTSAM_MODULE_PATH}")

if(GTSAM_UNSTABLE_BUILD_PYTHON)
set(ignore
        gtsam::Point2
        gtsam::Point3
        gtsam::LieVector
        gtsam::LieMatrix
        gtsam::ISAM2ThresholdMapValue
        gtsam::FactorIndices
        gtsam::FactorIndexSet
        gtsam::BetweenFactorPose3s
        gtsam::Point2Vector
        gtsam::Pose3Vector
        gtsam::KeyVector
        gtsam::FixedLagSmootherKeyTimestampMapValue
        gtsam::BinaryMeasurementsUnit3
        gtsam::KeyPairDoubleMap)
        
pybind_wrap(gtsam_unstable_py # target
        ${PROJECT_SOURCE_DIR}/gtsam_unstable/gtsam_unstable.i # interface_header
        "gtsam_unstable.cpp" # generated_cpp
        "gtsam_unstable" # module_name
        "gtsam" # top_namespace
        "${ignore}" # ignore_classes
        ${PROJECT_SOURCE_DIR}/python/gtsam_unstable/gtsam_unstable.tpl
        gtsam_unstable # libs
        "gtsam_unstable;gtsam_unstable_header" # dependencies
        ON # use_boost
        )

set_target_properties(gtsam_unstable_py PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
        OUTPUT_NAME "gtsam_unstable"
        LIBRARY_OUTPUT_DIRECTORY "${GTSAM_PYTHON_BUILD_DIRECTORY}/gtsam_unstable"
        DEBUG_POSTFIX "" # Otherwise you will have a wrong name
        RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
        )

set(GTSAM_UNSTABLE_MODULE_PATH ${GTSAM_PYTHON_BUILD_DIRECTORY}/gtsam_unstable)

# Symlink all tests .py files to build folder.
create_symlinks("${CMAKE_CURRENT_SOURCE_DIR}/gtsam_unstable"
        "${GTSAM_UNSTABLE_MODULE_PATH}")
endif()

set(GTSAM_PYTHON_INSTALL_TARGET python-install)
add_custom_target(${GTSAM_PYTHON_INSTALL_TARGET}
        COMMAND ${PYTHON_EXECUTABLE} ${GTSAM_PYTHON_BUILD_DIRECTORY}/setup.py install
        DEPENDS gtsam_py gtsam_unstable_py
        WORKING_DIRECTORY ${GTSAM_PYTHON_BUILD_DIRECTORY})

/**
 * @file    {module_name}.cpp
 * @brief   The auto-generated wrapper C++ source code.
 * @author  Duy-Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal
 * @date    Aug. 18, 2020
 *
 * ** THIS FILE IS AUTO-GENERATED, DO NOT MODIFY! **
 */

#define PYBIND11_DETAILED_ERROR_MESSAGES

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include "python/gtsam/optional_jacobian_pybind.h"
#include "gtsam/config.h"
#include "gtsam/base/serialization.h"
#include "gtsam/base/utilities.h"  // for RedirectCout.

// These are the included headers listed in `gtsam.i`
{includes}
#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/export.hpp>
#endif

// Export classes for serialization
{boost_class_export}

// Preamble for STL classes
#include "python/gtsam/preamble/{module_name}.h"

using namespace std;

namespace py = pybind11;

{submodules}

{declaration_module_def} {{
{wrapped_declarations}
}}

{binding_module_def} {{
// Specializations for STL classes
#include "python/gtsam/specializations/{module_name}.h"

{wrapped_bindings}
}}

{module_def} {{
    m_.doc() = "pybind11 wrapper of {module_name}";

{module_init}

    // Give the configured PIM backend its explicit Python name without
    // registering the same native C++ type twice with pybind11.
    if (py::hasattr(m_, "PreintegratedImuMeasurements")) {{
#if defined(GTSAM_LIEGROUP_PREINTEGRATION)
        m_.attr("PreintegratedImuMeasurementsLieGroup") =
            m_.attr("PreintegratedImuMeasurements");
#elif defined(GTSAM_TANGENT_PREINTEGRATION)
        m_.attr("PreintegratedImuMeasurementsTangent") =
            m_.attr("PreintegratedImuMeasurements");
#else
        m_.attr("PreintegratedImuMeasurementsManifold") =
            m_.attr("PreintegratedImuMeasurements");
#endif
    }}

}}

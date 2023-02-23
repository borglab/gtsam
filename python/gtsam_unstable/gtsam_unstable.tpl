/**
 * @file  gtsam.cpp
 * @brief   The auto-generated wrapper C++ source code.
 * @author  Duy-Nguyen Ta, Fan Jiang, Matthew Sklar
 * @date  Aug. 18, 2020
 *
 * ** THIS FILE IS AUTO-GENERATED, DO NOT MODIFY! **
 */

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include "gtsam/base/serialization.h"
#include "gtsam/base/utilities.h"  // for RedirectCout.

// These are the included headers listed in `gtsam_unstable.i`
{includes}
#include <boost/serialization/export.hpp>

{boost_class_export}

#include "python/gtsam_unstable/preamble.h"

using namespace std;

namespace py = pybind11;

PYBIND11_MODULE({module_name}, m_) {{
    m_.doc() = "pybind11 wrapper of {module_name}";

    // Note here we need to import the dependent library
    py::module::import("gtsam");

{wrapped_namespace}

#include "python/gtsam_unstable/specializations/gtsam_unstable.h"

}}


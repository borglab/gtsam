{include_boost}

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include "gtsam/base/serialization.h"
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

{includes}
#include <boost/serialization/export.hpp>

{boost_class_export}

{hoder_type}

#include "python/gtsam_unstable_py/preamble.h"

using namespace std;

namespace py = pybind11;

PYBIND11_MODULE({module_name}, m_) {{
    m_.doc() = "pybind11 wrapper of {module_name}";

    py::module::import("gtsam");

{wrapped_namespace}

#include "python/gtsam_unstable_py/specializations.h"

}}


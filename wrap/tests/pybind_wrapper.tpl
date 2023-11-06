#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

{includes}

{boost_class_export}

using namespace std;

namespace py = pybind11;

PYBIND11_MODULE({module_name}, m_) {{
    m_.doc() = "pybind11 wrapper of {module_name}";

{wrapped_namespace}

#include "python/specializations.h"

}}


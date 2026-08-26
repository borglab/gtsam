#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

{includes}

{boost_class_export}

using namespace std;

namespace py = pybind11;

{submodules}

{declaration_module_def} {{
{wrapped_declarations}
}}

{binding_module_def} {{
#include "python/specializations.h"
{wrapped_bindings}
}}

{module_def} {{
    m_.doc() = "pybind11 wrapper of {module_name}";

{module_init}
}}

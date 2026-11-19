/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/Errors.h>
#include <pybind11/stl.h>

namespace pybind11::detail {

/// Convert GTSAM's factor-associated vectors to and from native Python lists.
template <>
struct type_caster<gtsam::Errors> : list_caster<gtsam::Errors, gtsam::Vector> {
};

}  // namespace pybind11::detail

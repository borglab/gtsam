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
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include "gtsam/base/serialization.h"
#include "gtsam/base/utilities.h"  // for RedirectCout.

// These are the included headers listed in `gtsam_unstable.i`
{includes}
#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/export.hpp>
#include <gtsam/linear/NoiseModel.h>
#endif

{boost_class_export}

#if GTSAM_ENABLE_BOOST_SERIALIZATION
// Boost export registrations are local to each Python extension. Keep these
// dependencies synchronized with the serializable noise models in
// gtsam/linear/linear.i so unstable factors can serialize their polymorphic
// SharedNoiseModel members.
BOOST_CLASS_EXPORT(gtsam::noiseModel::Gaussian)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Diagonal)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Constrained)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Isotropic)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Unit)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Robust)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Null)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Fair)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Huber)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Cauchy)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Tukey)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Welsch)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::GemanMcClure)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::TruncatedLeastSquares)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::DCS)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::L2WithDeadZone)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::AsymmetricTukey)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::AsymmetricCauchy)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Custom)
#endif

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

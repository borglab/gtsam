#pragma once

#include <gtsam/config.h>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/export.hpp>
#include <gtsam/linear/NoiseModel.h>

// Every binding translation unit that serializes a noise model must see the
// same export key. Otherwise Boost can register a null key before linear.cpp's
// generated exports, making polymorphic archives depend on link/import order.
// Keep these keys synchronized with the serializable types in linear.i.
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Gaussian)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Diagonal)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Constrained)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Isotropic)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Unit)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::Robust)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Null)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Fair)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Huber)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Cauchy)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Tukey)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Welsch)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::GemanMcClure)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::TruncatedLeastSquares)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::DCS)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::L2WithDeadZone)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::AsymmetricTukey)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::AsymmetricCauchy)
BOOST_CLASS_EXPORT_KEY(gtsam::noiseModel::mEstimator::Custom)
#endif

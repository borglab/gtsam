/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VectorSerialization.h
 * @brief   serialization for Vectors
 * @author  Frank Dellaert
 * @date    February 2022
 */

#pragma once

#include <gtsam/base/Vector.h>

#include <boost/serialization/array.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {
namespace serialization {

// split version - copies into an STL vector for serialization
template <class Archive>
void save(Archive& ar, const gtsam::Vector& v, unsigned int /*version*/) {
  const size_t size = v.size();
  ar << BOOST_SERIALIZATION_NVP(size);
  ar << make_nvp("data", make_array(v.data(), v.size()));
}

template <class Archive>
void load(Archive& ar, gtsam::Vector& v, unsigned int /*version*/) {
  size_t size;
  ar >> BOOST_SERIALIZATION_NVP(size);
  v.resize(size);
  ar >> make_nvp("data", make_array(v.data(), v.size()));
}

// split version - copies into an STL vector for serialization
template <class Archive, int D>
void save(Archive& ar, const Eigen::Matrix<double, D, 1>& v,
          unsigned int /*version*/) {
  ar << make_nvp("data", make_array(v.data(), v.RowsAtCompileTime));
}

template <class Archive, int D>
void load(Archive& ar, Eigen::Matrix<double, D, 1>& v,
          unsigned int /*version*/) {
  ar >> make_nvp("data", make_array(v.data(), v.RowsAtCompileTime));
}

}  // namespace serialization
}  // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Vector)
BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Vector2)
BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Vector3)
BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Vector6)

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    MatrixSerialization.h
 * @brief   Serialization for matrices
 * @author  Frank Dellaert
 * @date    February 2022
 */

// \callgraph

#pragma once

#include <gtsam/base/Matrix.h>

#include <boost/serialization/array.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {
namespace serialization {

/**
 * Ref.
 * https://stackoverflow.com/questions/18382457/eigen-and-boostserialize/22903063#22903063
 *
 * Eigen supports calling resize() on both static and dynamic matrices.
 * This allows for a uniform API, with resize having no effect if the static
 * matrix is already the correct size.
 * https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html#TutorialMatrixSizesResizing
 *
 * We use all the Matrix template parameters to ensure wide compatibility.
 *
 * eigen_typekit in ROS uses the same code
 * http://docs.ros.org/lunar/api/eigen_typekit/html/eigen__mqueue_8cpp_source.html
 */

// split version - sends sizes ahead
template <class Archive, typename Scalar_, int Rows_, int Cols_, int Ops_,
          int MaxRows_, int MaxCols_>
void save(
    Archive& ar,
    const Eigen::Matrix<Scalar_, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& m,
    const unsigned int /*version*/) {
  const size_t rows = m.rows(), cols = m.cols();
  ar << BOOST_SERIALIZATION_NVP(rows);
  ar << BOOST_SERIALIZATION_NVP(cols);
  ar << make_nvp("data", make_array(m.data(), m.size()));
}

template <class Archive, typename Scalar_, int Rows_, int Cols_, int Ops_,
          int MaxRows_, int MaxCols_>
void load(Archive& ar,
          Eigen::Matrix<Scalar_, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& m,
          const unsigned int /*version*/) {
  size_t rows, cols;
  ar >> BOOST_SERIALIZATION_NVP(rows);
  ar >> BOOST_SERIALIZATION_NVP(cols);
  m.resize(rows, cols);
  ar >> make_nvp("data", make_array(m.data(), m.size()));
}

// templated version of BOOST_SERIALIZATION_SPLIT_FREE(Eigen::Matrix);
template <class Archive, typename Scalar_, int Rows_, int Cols_, int Ops_,
          int MaxRows_, int MaxCols_>
void serialize(
    Archive& ar,
    Eigen::Matrix<Scalar_, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& m,
    const unsigned int version) {
  split_free(ar, m, version);
}

// specialized to Matrix for MATLAB wrapper
template <class Archive>
void serialize(Archive& ar, gtsam::Matrix& m, const unsigned int version) {
  split_free(ar, m, version);
}

}  // namespace serialization
}  // namespace boost

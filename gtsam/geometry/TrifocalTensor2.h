/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    TrifocalTensor2.h
 * @brief   A 2x2x2 trifocal tensor in a plane, for 1D cameras.
 * @author  Zhaodong Yang
 * @author  Akshay Krishnan
 */
// \callgraph

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>

namespace gtsam {

/**
 * @brief A trifocal tensor for 1D cameras in a plane. It encodes the
 * relationship between bearing measurements of a point in the plane observed in
 * 3 1D cameras.
 * @addtogroup geometry
 * \nosubgrouping
 */
class TrifocalTensor2 {
 private:
  // The trifocal tensor has 2 matrices.
  Matrix2 matrix0_, matrix1_;

 public:
  TrifocalTensor2() {}

  // Construct from the two 2x2 matrices that form the tensor.
  TrifocalTensor2(const Matrix2& matrix0, const Matrix2& matrix1)
      : matrix0_(matrix0), matrix1_(matrix1) {}

  /**
   * @brief Estimates a tensor from 8 bearing measurements in 3 cameras. Throws
   * a runtime error if the size of inputs are unequal or less than 8.
   *
   * @param u bearing measurement in camera u.
   * @param v bearing measurement in camera v.
   * @param w bearing measurement in camera w.
   * @return Tensor estimated from the measurements.
   */
  static TrifocalTensor2 FromBearingMeasurements(
      const std::vector<Rot2>& bearings_u, const std::vector<Rot2>& bearings_v,
      const std::vector<Rot2>& bearings_w);

  /**
   * @brief Estimates a tensor from 8 projective measurements in 3 cameras.
   * Throws a runtime error if the size of inputs are unequal or less than 8.
   *
   * @param u projective 1D bearing measurement in camera u.
   * @param v projective 1D bearing measurement in camera v.
   * @param w projective 1D bearing measurement in camera w.
   * @return tensor estimated from the measurements.
   */
  static TrifocalTensor2 FromProjectiveBearingMeasurements(
      const std::vector<Point2>& u, const std::vector<Point2>& v,
      const std::vector<Point2>& w);

  /**
   * @brief Computes the bearing in camera 'u' given bearing measurements in
   * cameras 'v' and 'w'.
   *
   * @param vZp bearing measurement in camera v
   * @param wZp bearing measurement in camera w
   * @return bearing measurement in camera u
   */
  Rot2 transform(const Rot2& vZp, const Rot2& wZp) const;

  /**
   * @brief Computes the bearing in camera 'u' from that of cameras 'v' and 'w',
   * in projective coordinates.
   *
   * @param vZp projective bearing measurement in camera v
   * @param wZp projective bearing measurement in camera w
   * @return projective bearing measurement in camera u
   */
  Point2 transform(const Point2& vZp, const Point2& wZp) const;

  // Accessors for the two matrices that comprise the trifocal tensor.
  Matrix2 mat0() const { return matrix0_; }
  Matrix2 mat1() const { return matrix1_; }
};

}  // namespace gtsam

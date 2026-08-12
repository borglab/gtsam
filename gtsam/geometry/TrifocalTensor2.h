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
#include <gtsam/base/Testable.h>
#include <gtsam/dllexport.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>

#include <string>
#include <vector>

namespace gtsam {

/**
 * @brief A trifocal tensor for 1D cameras in a plane. It encodes the
 * relationship between bearing measurements of a point in the plane observed in
 * 3 1D cameras. The tensor is homogeneous, so tensors that differ only by a
 * nonzero scale represent the same geometry.
 *
 * This class supports linear estimation and transfer. It deliberately does not
 * define a nonlinear manifold chart; adding one requires choosing a
 * scale-invariant parameterization for the tensor.
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT TrifocalTensor2 {
 private:
  // The trifocal tensor has 2 matrices.
  Matrix2 matrix0_, matrix1_;

 public:
  /// Construct the zero tensor.
  TrifocalTensor2() : matrix0_(Matrix2::Zero()), matrix1_(Matrix2::Zero()) {}

  /// Construct from the two 2x2 matrices that form the tensor.
  TrifocalTensor2(const Matrix2& matrix0, const Matrix2& matrix1)
      : matrix0_(matrix0), matrix1_(matrix1) {}

  /**
   * Estimates a tensor from at least seven corresponding bearing measurements
   * in three cameras. Throws std::invalid_argument if the input sizes differ or
   * fewer than seven correspondences are supplied.
   */
  static TrifocalTensor2 FromBearingMeasurements(
      const std::vector<Rot2>& bearings_u, const std::vector<Rot2>& bearings_v,
      const std::vector<Rot2>& bearings_w);

  /**
   * Estimates a tensor from at least seven corresponding projective bearing
   * measurements in three cameras. Throws std::invalid_argument if the input
   * sizes differ or fewer than seven correspondences are supplied.
   */
  static TrifocalTensor2 FromProjectiveBearingMeasurements(
      const std::vector<Point2>& u, const std::vector<Point2>& v,
      const std::vector<Point2>& w);

  /**
   * Computes the bearing in camera u from bearing measurements in cameras v
   * and w.
   */
  Rot2 transform(const Rot2& vZp, const Rot2& wZp) const;

  /**
   * Computes the projective bearing in camera u from projective bearing
   * measurements in cameras v and w.
   */
  Point2 transform(const Point2& vZp, const Point2& wZp) const;

  /// Print the two tensor matrices.
  void print(const std::string& s = "") const;

  /// Check equality up to the homogeneous tensor scale.
  bool equals(const TrifocalTensor2& other, double tol = 1e-9) const;

  /// Access the first matrix comprising the trifocal tensor.
  Matrix2 mat0() const { return matrix0_; }

  /// Access the second matrix comprising the trifocal tensor.
  Matrix2 mat1() const { return matrix1_; }
};

template <>
struct traits<TrifocalTensor2> : public Testable<TrifocalTensor2> {};

template <>
struct traits<const TrifocalTensor2> : public Testable<TrifocalTensor2> {};

}  // namespace gtsam

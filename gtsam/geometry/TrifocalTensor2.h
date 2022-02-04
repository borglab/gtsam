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
#include <gtsam/geometry/Rot2.h>

namespace gtsam {

class TrifocalTensor2 {
 private:
  Matrix2 matrix0_, matrix1_;

 public:
  TrifocalTensor2() {}

  TrifocalTensor2(const Matrix2& matrix0, const Matrix2& matrix1);

  TrifocalTensor2(const std::vector<Rot2>& bearings_u,
                  const std::vector<Rot2>& bearings_v,
                  const std::vector<Rot2>& bearings_w) const;
  
  Rot2 transform(const Rot2& vZp, const Rot2& wZp) const;

  Point2 transform(const Point2& vZp, const Point2& wZp) const;
};

}  // namespace gtsam
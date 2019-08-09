/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EigenOptimizer.h
 *
 * @brief optimizer linear factor graph using eigen solver as backend
 *
 * @date Aug 2019
 * @author Mandy Xie
 */
#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <Eigen/Sparse>
#include <string>

namespace gtsam {
/// Optimize using Eigen's SparseQR factorization
VectorValues optimizeEigenQR(const GaussianFactorGraph &gfg,
                             const std::string &orderingType);

/// Optimize using Eigen's SimplicailLDLT factorization
VectorValues optimizeEigenCholesky(const GaussianFactorGraph &gfg,
                                   const std::string &orderingType);

}  // namespace gtsam

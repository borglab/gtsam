/*
 * @file  JacobianFactorQR.h
 * @brief Jacobianfactor that combines and eliminates points
 * @date  Oct 27, 2013
 * @uthor Frank Dellaert
 */

#pragma once
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/RegularJacobianFactor.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam {

class GaussianBayesNet;

/**
 * JacobianFactor for Schur complement that uses Q noise model
 */
template<size_t D, size_t ZDim>
class JacobianFactorQR: public RegularJacobianFactor<D> {

  typedef RegularJacobianFactor<D> Base;
  typedef Eigen::Matrix<double, ZDim, D> MatrixZD;

public:

  /**
   * Constructor
   */
  JacobianFactorQR(const KeyVector& keys,
      const std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD> >& FBlocks, const Matrix& E, const Matrix3& P,
      const Vector& b, //
      const SharedDiagonal& model = SharedDiagonal()) :
      Base() {
    // Create a number of Jacobian factors in a factor graph
    GaussianFactorGraph gfg;
    Symbol pointKey('p', 0);
    for (size_t k = 0; k < FBlocks.size(); ++k) {
      Key key = keys[k];
      gfg.add(pointKey, E.block<ZDim, 3>(ZDim * k, 0), key, FBlocks[k],
          b.segment < ZDim > (ZDim * k), model);
    }
    //gfg.print("gfg");

    // eliminate the point
    std::shared_ptr<GaussianBayesNet> bn;
    GaussianFactorGraph::shared_ptr fg;
    KeyVector variables;
    variables.push_back(pointKey);
    boost::tie(bn, fg) = gfg.eliminatePartialSequential(variables, EliminateQR);
    //fg->print("fg");

    JacobianFactor::operator=(JacobianFactor(*fg));
  }
};
// end class JacobianFactorQR

}// end namespace gtsam

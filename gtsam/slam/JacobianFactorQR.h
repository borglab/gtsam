/*
 * @file  JacobianFactorQR.h
 * @brief Jacobianfactor that combines and eliminates points
 * @date  Oct 27, 2013
 * @uthor Frank Dellaert
 */

#pragma once
#include <gtsam/slam/JacobianSchurFactor.h>

namespace gtsam {
/**
 * JacobianFactor for Schur complement that uses Q noise model
 */
template<size_t D, size_t ZDim>
class JacobianFactorQR: public JacobianSchurFactor<D, ZDim> {

  typedef JacobianSchurFactor<D, ZDim> Base;

public:

  /**
   * Constructor
   */
  JacobianFactorQR(const std::vector<typename Base::KeyMatrix2D>& Fblocks,
      const Matrix& E, const Matrix3& P, const Vector& b,
      const SharedDiagonal& model = SharedDiagonal()) :
      JacobianSchurFactor<D, ZDim>() {
    // Create a number of Jacobian factors in a factor graph
    GaussianFactorGraph gfg;
    Symbol pointKey('p', 0);
    size_t i = 0;
    BOOST_FOREACH(const typename Base::KeyMatrix2D& it, Fblocks) {
      gfg.add(pointKey, E.block<ZDim, 3>(ZDim * i, 0), it.first, it.second,
          b.segment<ZDim>(ZDim * i), model);
      i += 1;
    }
    //gfg.print("gfg");

    // eliminate the point
    GaussianBayesNet::shared_ptr bn;
    GaussianFactorGraph::shared_ptr fg;
    std::vector < Key > variables;
    variables.push_back(pointKey);
    boost::tie(bn, fg) = gfg.eliminatePartialSequential(variables, EliminateQR);
    //fg->print("fg");

    JacobianFactor::operator=(JacobianFactor(*fg));
  }
};
// class

}// gtsam

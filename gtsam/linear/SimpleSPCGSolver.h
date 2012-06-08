/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/ConjugateGradientSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

/**
 * This class gives a simple implementation to the SPCG solver presented in Dellaert et al in IROS'10.
 *
 * Given a linear least-squares problem \f$ f(x) = |A x - b|^2 \f$. We split the problem into
 * \f$ f(x) = |A_t - b_t|^2 + |A_c - b_c|^2 \f$ where \f$ A_t \f$ denotes the "tree" part, and \f$ A_c \f$ denotes the "constraint" part.
 * \f$ A_t \f$ is factorized into \f$ Q_t R_t \f$, and we compute \f$ c_t = Q_t^{-1} b_t \f$, and \f$ x_t = R_t^{-1} c_t \f$ accordingly.
 * Then we solve a reparametrized problem \f$ f(y) = |y|^2 + |A_c R_t^{-1} y - \bar{b_y}|^2 \f$, where \f$ y = R_t(x - x_t) \f$, and \f$ \bar{b_y} = (b_c - A_c x_t) \f$
 *
 * In the matrix form, it is equivalent to solving \f$ [A_c R_t^{-1} ; I ] y = [\bar{b_y} ; 0] \f$. We can solve it
 * with the least-squares variation of the conjugate gradient method.
 *
 * Note: A full SPCG implementation will come up soon in the next release.
 *
 * \nosubgrouping
 */

class SimpleSPCGSolver : public IterativeSolver {

public:

  typedef IterativeSolver Base;
  typedef ConjugateGradientParameters Parameters;
  typedef boost::shared_ptr<IterativeSolver> shared_ptr;

protected:

  size_t nVar_ ;                                ///< number of variables \f$ x \f$
  size_t nAc_ ;                                 ///< number of factors in \f$ A_c \f$
  FactorGraph<JacobianFactor>::shared_ptr Ac_;  ///< the constrained part of the graph
  GaussianBayesNet::shared_ptr Rt_;             ///< the gaussian bayes net of the tree part of the graph
  VectorValues::shared_ptr xt_;                 ///< the solution of the \f$ A_t^{-1} b_t \f$
  VectorValues::shared_ptr y0_;                 ///< a column zero vector
  VectorValues::shared_ptr by_;                 ///< \f$ [\bar{b_y} ; 0 ] \f$
  VectorValues::shared_ptr tmpY_;               ///< buffer for the column vectors
  VectorValues::shared_ptr tmpB_;               ///< buffer for the row vectors
  Parameters parameters_;                       ///< Parameters for iterative method

public:

  SimpleSPCGSolver(const GaussianFactorGraph &gfg, const Parameters &parameters);
  virtual ~SimpleSPCGSolver() {}
  virtual VectorValues::shared_ptr optimize () {return optimize(*y0_);}

protected:

  VectorValues::shared_ptr optimize (const VectorValues &initial);

  /** output = \f$ [\bar{b_y} ; 0 ] - [A_c R_t^{-1} ; I] \f$ input */
  void residual(const VectorValues &input, VectorValues &output);

  /** output = \f$ [A_c R_t^{-1} ; I] \f$ input */
  void multiply(const VectorValues &input, VectorValues &output);

  /** output = \f$ [R_t^{-T} A_c^T,  I] \f$ input */
  void transposeMultiply(const VectorValues &input, VectorValues &output);

  /** output = \f$ R_t^{-1} \f$ input */
  void backSubstitute(const VectorValues &rhs, VectorValues &sol) ;

  /** output = \f$ R_t^{-T} \f$ input */
  void transposeBackSubstitute(const VectorValues &rhs, VectorValues &sol) ;

  /** return \f$ R_t^{-1} y + x_t \f$ */
  VectorValues::shared_ptr transform(const VectorValues &y);

  /** naively split a gaussian factor graph into tree and constraint parts
   * Note: This function has to be refined for your graph/application */
  boost::tuple<GaussianFactorGraph::shared_ptr, FactorGraph<JacobianFactor>::shared_ptr>
  splitGraph(const GaussianFactorGraph &gfg);
};

}

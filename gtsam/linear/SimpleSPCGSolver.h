/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

/**
 * This class gives a simple implementation to the SPCG solver presented in Dellaert et al. IROS'10.
 * Given a linear least-squares problem f(x) = |A x - b|^2.
 * We split the problem into f(x) = |At - bt|^2 + |Ac - bc|^2 where At denotes the "tree" part,
 * and Ac denotes the "constraint" part. At is factorized into Qt Rt, and we compute ct = Qt\bt, and xt = Rt\ct.
 * Then we solve reparametrized problem f(y) = |y|^2 + |Ac Rt \ y - by|^2, where y = Rt(x - xt), and by = (bc - Ac xt)
 * In the matrix form, it is equivalent to solving [Ac Rt^{-1} ; I ] y = [by ; 0].
 */

class SimpleSPCGSolver : public IterativeSolver {

public:

  typedef IterativeSolver Base;
  typedef boost::shared_ptr<IterativeSolver> shared_ptr;

protected:

  size_t nVar_ ; /* number of variables */
  size_t nAc_ ;  /* number of factors in Ac */

  /* for SPCG */
  FactorGraph<JacobianFactor>::shared_ptr Ac_;
  GaussianBayesNet::shared_ptr Rt_;
  VectorValues::shared_ptr xt_;
  VectorValues::shared_ptr y0_;
  VectorValues::shared_ptr by_;   /* [bc_bar ; 0 ] */

  /* buffer for row and column vectors in */
  VectorValues::shared_ptr tmpY_;
  VectorValues::shared_ptr tmpB_;

public:

  SimpleSPCGSolver(const GaussianFactorGraph &gfg, const Parameters::shared_ptr &parameters);
  virtual ~SimpleSPCGSolver() {}
  virtual VectorValues::shared_ptr optimize () {return optimize(*y0_);}

protected:

  VectorValues::shared_ptr optimize (const VectorValues &initial);

  /* output = [bc_bar ; 0 ] - [Ac Rt^{-1} ; I] y */
  void residual(const VectorValues &input, VectorValues &output);

  /* output = [Ac Rt^{-1} ; I] input */
  void multiply(const VectorValues &input, VectorValues &output);

  /* output = [Rt^{-T} Ac^T,  I] input */
  void transposeMultiply(const VectorValues &input, VectorValues &output);

  /* output = Rt^{-1} input */
  void backSubstitute(const VectorValues &rhs, VectorValues &sol) ;

  /* output = Rt^{-T} input */
  void transposeBackSubstitute(const VectorValues &rhs, VectorValues &sol) ;

  /* return x = Rt^{-1} y + x_t */
  VectorValues::shared_ptr transform(const VectorValues &y);

  /* naively split a gaussian factor graph into tree and constraint parts
   * Note: This function has to be refined for your graph/application */
  boost::tuple<GaussianFactorGraph::shared_ptr, FactorGraph<JacobianFactor>::shared_ptr>
  splitGraph(const GaussianFactorGraph &gfg);

};

}

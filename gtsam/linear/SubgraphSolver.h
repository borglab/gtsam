/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SubgraphSolver.h
 * @brief  Subgraph Solver from IROS 2010
 * @date   2010
 * @author Frank Dellaert
 * @author Yong Dian Jian
 */

#pragma once

#include <gtsam/linear/ConjugateGradientSolver.h>
#include <gtsam/linear/SubgraphBuilder.h>

#include <map>
#include <utility>  // pair

namespace gtsam {

// Forward declarations
class GaussianFactorGraph;
class GaussianBayesNet;
class SubgraphPreconditioner;

struct GTSAM_EXPORT SubgraphSolverParameters
    : public ConjugateGradientParameters {
  SubgraphBuilderParameters builderParams;
  explicit SubgraphSolverParameters(const SubgraphBuilderParameters &p = SubgraphBuilderParameters())
    : builderParams(p) {}
  void print() const { Base::print(); }
  void print(std::ostream &os) const override {
    Base::print(os);
  }
};

/**
 * This class implements the linear SPCG solver presented in Dellaert et al in
 * IROS'10.
 *
 * Given a linear least-squares problem \f$ f(x) = |A x - b|^2 \f$. We split the
 * problem into \f$ f(x) = |A_t - b_t|^2 + |A_c - b_c|^2 \f$ where \f$ A_t \f$
 * denotes the "tree" part, and \f$ A_c \f$ denotes the "constraint" part.
 *
 * \f$A_t \f$ is factorized into \f$ Q_t R_t \f$, and we compute
 * \f$ c_t = Q_t^{-1} b_t \f$, and \f$ x_t = R_t^{-1} c_t \f$ accordingly.
 *
 * Then we solve a reparametrized problem
 * \f$ f(y) = |y|^2 + |A_c R_t^{-1} y -\bar{b_y}|^2 \f$,
 * where \f$ y = R_t(x - x_t) \f$, and \f$ \bar{b_y} = (b_c - A_c x_t) \f$
 *
 * In the matrix form, it is equivalent to solving
 * \f$ [A_c R_t^{-1} ; I ] y = [\bar{b_y} ; 0] \f$.
 * We can solve it with the least-squares variation of the conjugate gradient
 * method.
 *
 * To use it in nonlinear optimization, please see the following example
 *
 *  LevenbergMarquardtParams parameters;
 *  parameters.linearSolverType = NonlinearOptimizerParams::CONJUGATE_GRADIENT;
 *  parameters.iterativeParams = boost::make_shared<SubgraphSolverParameters>();
 *  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);
 *  Values result = optimizer.optimize();
 *
 * \nosubgrouping
 */
class GTSAM_EXPORT SubgraphSolver : public IterativeSolver {
 public:
  typedef SubgraphSolverParameters Parameters;

 protected:
  Parameters parameters_;
  boost::shared_ptr<SubgraphPreconditioner> pc_;  ///< preconditioner object

 public:
  /// @name Constructors
  /// @{
  /**
   * Given a gaussian factor graph, split it into a spanning tree (A1) + others
   * (A2) for SPCG Will throw exception if there are ternary factors or higher
   * arity, as we use Kruskal's algorithm to split the graph, treating binary
   * factors as edges.
   */
  SubgraphSolver(const GaussianFactorGraph &A, const Parameters &parameters,
                 const Ordering &ordering);

  /**
   * The user specifies the subgraph part and the constraints part.
   * May throw exception if A1 is underdetermined. An ordering is required to
   * eliminate Ab1. We take Ab1 as a const reference, as it will be transformed
   * into Rc1, but take Ab2 as a shared pointer as we need to keep it around.
   */
  SubgraphSolver(const GaussianFactorGraph &Ab1,
                 const boost::shared_ptr<GaussianFactorGraph> &Ab2,
                 const Parameters &parameters, const Ordering &ordering);
  /**
   * The same as above, but we assume A1 was solved by caller.
   * We take two shared pointers as we keep both around.
   */
  SubgraphSolver(const boost::shared_ptr<GaussianBayesNet> &Rc1,
                 const boost::shared_ptr<GaussianFactorGraph> &Ab2,
                 const Parameters &parameters);

  /// Destructor
  ~SubgraphSolver() override {}

  /// @}
  /// @name Implement interface
  /// @{

  /// Optimize from zero
  VectorValues optimize() const;

  /// Interface that IterativeSolver subclasses have to implement
  VectorValues optimize(const GaussianFactorGraph &gfg,
                        const KeyInfo &keyInfo,
                        const std::map<Key, Vector> &lambda,
                        const VectorValues &initial) override;

  /// @}
  /// @name Implement interface
  /// @{

  /// Split graph using Kruskal algorithm, treating binary factors as edges.
  std::pair < boost::shared_ptr<GaussianFactorGraph>,
      boost::shared_ptr<GaussianFactorGraph> > splitGraph(
          const GaussianFactorGraph &gfg);

  /// @}
};

}  // namespace gtsam

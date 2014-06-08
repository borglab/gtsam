/*
 * PCGSolver.h
 *
 *  Created on: Jan 31, 2012
 *  Author: Yong-Dian Jian
 */

#pragma once

#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/ConjugateGradientSolver.h>
#include <gtsam/linear/VectorValues.h>
#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <iosfwd>
#include <map>
#include <string>

namespace gtsam {

class GaussianFactorGraph;
class KeyInfo;
class Preconditioner;
class PreconditionerParameters;

/*****************************************************************************/
struct PCGSolverParameters: public ConjugateGradientParameters {
public:
  typedef ConjugateGradientParameters Base;
  typedef boost::shared_ptr<PCGSolverParameters> shared_ptr;

  PCGSolverParameters() {}

  virtual void print(std::ostream &os) const;

  /* interface to preconditioner parameters */
  inline const PreconditionerParameters& preconditioner() const {
    return *preconditioner_;
  }

  boost::shared_ptr<PreconditionerParameters> preconditioner_;
};

/*****************************************************************************/
/* A virtual base class for the preconditioned conjugate gradient solver */
class PCGSolver: public IterativeSolver {
public:
  typedef IterativeSolver Base;
  typedef boost::shared_ptr<PCGSolver> shared_ptr;

protected:

  PCGSolverParameters parameters_;
  boost::shared_ptr<Preconditioner> preconditioner_;

//  /* cached local variables */
//  KeyInfo keyInfo_;
//
//  /* whether the preconditioner has be built */
//  bool built_ ;

public:
  /* Interface to initialize a solver without a problem */
  PCGSolver(const PCGSolverParameters &p);
  virtual ~PCGSolver() {}

//  /* update interface to the nonlinear optimizer  */
//  virtual void replaceFactors(
//      const GaussianFactorGraph &factorGraph,
//      const Values &linearization_point,
//      const std::map<Key, double> lambda);

//  /* build the preconditioner */
//  void buildPreconditioner();

//  /* interface for the NonlinearOptimizer, with initial estimate equals to a zero vector */
//  virtual VectorValues optimize() ;
//
//  /* interface for the NonlinearOptimizer, with a custom initial estimate*/
//  virtual VectorValues optimize(const VectorValues &initial);

  using IterativeSolver::optimize;

  virtual VectorValues optimize(const GaussianFactorGraph &gfg,
      const KeyInfo &keyInfo, const std::map<Key, Vector> &lambda,
      const VectorValues &initial);

//protected:

//  /* convert jacobian factor graph to solver-specific kernel */
//  virtual void convertKernel(const JacobianFactorGraph &jfg, const double lambda = 0.0) = 0 ;
//
//  /* do the actual optimization */
//  virtual VectorValues optimize_(const VectorValues &initial) = 0;

};

/*****************************************************************************/
class GaussianFactorGraphSystem {
public:

  GaussianFactorGraphSystem(const GaussianFactorGraph &gfg,
      const Preconditioner &preconditioner, const KeyInfo &info,
      const std::map<Key, Vector> &lambda);

  const GaussianFactorGraph &gfg_;
  const Preconditioner &preconditioner_;
  const KeyInfo &keyInfo_;
  const std::map<Key, Vector> &lambda_;

  void residual(const Vector &x, Vector &r) const;
  void multiply(const Vector &x, Vector& y) const;
  void leftPrecondition(const Vector &x, Vector &y) const;
  void rightPrecondition(const Vector &x, Vector &y) const;
  inline void scal(const double alpha, Vector &x) const {
    x *= alpha;
  }
  inline double dot(const Vector &x, const Vector &y) const {
    return x.dot(y);
  }
  inline void axpy(const double alpha, const Vector &x, Vector &y) const {
    y += alpha * x;
  }

  void getb(Vector &b) const;
};

/* utility functions */
/**********************************************************************************/
VectorValues buildVectorValues(const Vector &v, const Ordering &ordering,
    const std::map<Key, size_t> & dimensions);

/**********************************************************************************/
VectorValues buildVectorValues(const Vector &v, const KeyInfo &keyInfo);

///*****************************************************************************/
///* an specialization of the PCGSolver using gtsam's factor graph and linear algebra facility */
//class PCGSolver_FG : public PCGSolver {
//
//public:
//  typedef PCGSolver Base;
//  typedef boost::shared_ptr<PCGSolver_FG> shared_ptr ;
//
//protected:
//  JacobianFactorGraph::shared_ptr hfg_; // A'*A + lambda*I
//
//public:
//  PCGSolver_FG(const Parameters &parameters) : Base(parameters) {}
//  virtual ~PCGSolver_FG() {}
//
//protected:
//  virtual void convertKernel(const JacobianFactorGraph &jfg, const double lambda = 0.0);
//  virtual VectorValues optimize_(const VectorValues &initial);
//
//  /* interface to the preconditionedConjugateGradient function template */
//  class System {
//  public:
//    typedef Vector State;
//    typedef Vector Residual;
//
//  protected:
//    const JacobianFactorGraph::shared_ptr hfg_;
//    const Preconditioner::shared_ptr preconditioner_;
//    const KeyInfo &keyInfo_;
//    const RowInfo &rowInfo_;
//
//  public:
//    System(const JacobianFactorGraph::shared_ptr hfg,
//        const Preconditioner::shared_ptr preconditioner,
//        const KeyInfo &keyInfo, const RowInfo &rowInfo);
//
//    inline void residual(const State &x, State &r) const { gtsam::residual(*hfg_, x, r); }
//    inline void multiply(const State &x, State& y) const { gtsam::multiply(*hfg_, x, y); }
//    inline void leftPrecondition(const State &x, State &y) const { preconditioner_->transposeSolve(x, y); }
//    inline void rightPrecondition(const State &x, State &y) const { preconditioner_->solve(x, y); }
//    inline void scal(const double alpha, State &x) const { x.vector() *= alpha;  }
//    inline double dot(const State &x, const State &y) const { return x.vector().dot(y.vector()); }
//    inline void axpy(const double alpha, const State &x, State &y) const { y.vector() += alpha*x.vector(); }
//  } ;
//};
//
///*****************************************************************************/
///** an specialization of the PCGSolver using sbm and customized blas kernel **/
//class PCGSolver_SBM : public PCGSolver {
//
//public:
//  typedef PCGSolver Base;
//  typedef boost::shared_ptr<PCGSolver_SBM> shared_ptr ;
//
//protected:
//  ydjian::SparseLinearSystem::shared_ptr linear_;
//
//public:
//  PCGSolver_SBM(const Parameters &parameters) : Base(parameters) {}
//  virtual ~PCGSolver_SBM() {}
//
//protected:
//  /* virtual function of the PCGSolver */
//  virtual void convertKernel(const JacobianFactorGraph &jfg, const double lambda = 0.0);
//  virtual VectorValues optimize_(const VectorValues &initial);
//
//  /* interface to the preconditionedConjugateGradient function template */
//  class System {
//
//  public:
//    typedef Vector State;
//
//  protected:
//    const ydjian::SparseLinearSystem::shared_ptr linear_;
//    const Preconditioner::shared_ptr preconditioner_;
//
//  public:
//    System(const ydjian::SparseLinearSystem::shared_ptr linear, const Preconditioner::shared_ptr preconditioner)
//      : linear_(linear), preconditioner_(preconditioner) {}
//    inline void residual(const State &x, State &r) const { linear_->residual(x.data(), r.data(), false); }
//    inline void multiply(const State &x, State& y) const { linear_->matrix()->multiply(x.data(), y.data(), false); }
//    inline void leftPrecondition(const State &x, State &y) const { preconditioner_->transposeSolve(x, y); }
//    inline void rightPrecondition(const State &x, State &y) const { preconditioner_->solve(x, y); }
//    inline void scal(const double alpha, State &x) const { x *= alpha;  }
//    inline double dot(const State &x, const State &y) const { return x.dot(y); }
//    inline void axpy(const double alpha, const State &x, State &y) const { y += alpha*x; }
//  } ;
//};
//
///* a factory method to instantiate PCGSolvers and its derivatives */
//PCGSolver::shared_ptr createPCGSolver(const PCGParameters &parameters);
//
///* an utility function to create sparse linear system, for all cg solvers using the sbm kernel */
//ydjian::SparseLinearSystem::shared_ptr
//buildSparseLinearSystem(const JacobianFactorGraph &jfg, const KeyInfo &keyInfo, const bool AtA,
//  const double lambda, const bool colMajor, const PCGParameters::BLASKernel blas,
//  const PCGParameters::RegisterBlockKernel rb);

}


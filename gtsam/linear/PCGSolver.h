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
struct PreconditionerParameters;

/*****************************************************************************/
struct GTSAM_EXPORT PCGSolverParameters: public ConjugateGradientParameters {
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
class GTSAM_EXPORT PCGSolver: public IterativeSolver {
public:
  typedef IterativeSolver Base;
  typedef boost::shared_ptr<PCGSolver> shared_ptr;

protected:

  PCGSolverParameters parameters_;
  boost::shared_ptr<Preconditioner> preconditioner_;

public:
  /* Interface to initialize a solver without a problem */
  PCGSolver(const PCGSolverParameters &p);
  virtual ~PCGSolver() {}

  using IterativeSolver::optimize;

  virtual VectorValues optimize(const GaussianFactorGraph &gfg,
      const KeyInfo &keyInfo, const std::map<Key, Vector> &lambda,
      const VectorValues &initial);

};

/*****************************************************************************/
class GTSAM_EXPORT GaussianFactorGraphSystem {
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

}


/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file PCGSolver.h
 * @brief Preconditioned Conjugate Gradient Solver for linear systems
 * @date Jan 31, 2012
 * @author Yong-Dian Jian
 * @author Sungtae An
 */

#pragma once

#include <gtsam/linear/ConjugateGradientSolver.h>
#include <string>

namespace gtsam {

class GaussianFactorGraph;
class KeyInfo;
class Preconditioner;
class VectorValues;
struct PreconditionerParameters;

/**
 * Parameters for PCG
 */
struct GTSAM_EXPORT PCGSolverParameters: public ConjugateGradientParameters {
public:
  typedef ConjugateGradientParameters Base;
  typedef std::shared_ptr<PCGSolverParameters> shared_ptr;

  PCGSolverParameters() {
  }

  void print(std::ostream &os) const override;

  /* interface to preconditioner parameters */
  inline const PreconditionerParameters& preconditioner() const {
    return *preconditioner_;
  }

  // needed for python wrapper
  void print(const std::string &s) const;

  std::shared_ptr<PreconditionerParameters> preconditioner_;

  void setPreconditionerParams(const std::shared_ptr<PreconditionerParameters> preconditioner);
};

/**
 * A virtual base class for the preconditioned conjugate gradient solver
 */
class GTSAM_EXPORT PCGSolver: public IterativeSolver {
public:
  typedef IterativeSolver Base;
  typedef std::shared_ptr<PCGSolver> shared_ptr;

protected:

  PCGSolverParameters parameters_;
  std::shared_ptr<Preconditioner> preconditioner_;

public:
  /* Interface to initialize a solver without a problem */
  PCGSolver(const PCGSolverParameters &p);
  ~PCGSolver() override {
  }

  using IterativeSolver::optimize;

  VectorValues optimize(const GaussianFactorGraph &gfg,
      const KeyInfo &keyInfo, const std::map<Key, Vector> &lambda,
      const VectorValues &initial) override;

};

/**
 * System class needed for calling preconditionedConjugateGradient
 */
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
  void scal(const double alpha, Vector &x) const;
  double dot(const Vector &x, const Vector &y) const;
  void axpy(const double alpha, const Vector &x, Vector &y) const;

  void getb(Vector &b) const;
};

/// @name utility functions
/// @{

/// Create VectorValues from a Vector
VectorValues buildVectorValues(const Vector &v, const Ordering &ordering,
    const std::map<Key, size_t> & dimensions);

/// Create VectorValues from a Vector and a KeyInfo class
VectorValues buildVectorValues(const Vector &v, const KeyInfo &keyInfo);

/// @}

}


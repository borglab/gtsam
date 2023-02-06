/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file PCGSolver.cpp
 * @brief Preconditioned Conjugate Gradient Solver for linear systems
 * @date Feb 14, 2012
 * @author Yong-Dian Jian
 * @author Sungtae An
 */

#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/VectorValues.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

using namespace std;

namespace gtsam {

/*****************************************************************************/
void PCGSolverParameters::print(ostream &os) const {
  Base::print(os);
  os << "PCGSolverParameters:" << endl;
  preconditioner_->print(os);
}

/*****************************************************************************/
PCGSolver::PCGSolver(const PCGSolverParameters &p) {
  parameters_ = p;
  preconditioner_ = createPreconditioner(p.preconditioner_);
}

void PCGSolverParameters::setPreconditionerParams(const std::shared_ptr<PreconditionerParameters> preconditioner) {
  preconditioner_ = preconditioner;
}

void PCGSolverParameters::print(const std::string &s) const {
  std::cout << s << std::endl;;
  std::ostringstream os;
  print(os);
  std::cout << os.str() << std::endl;
}

/*****************************************************************************/
VectorValues PCGSolver::optimize(const GaussianFactorGraph &gfg,
    const KeyInfo &keyInfo, const std::map<Key, Vector> &lambda,
    const VectorValues &initial) {
  /* build preconditioner */
  preconditioner_->build(gfg, keyInfo, lambda);

  /* apply pcg */
  GaussianFactorGraphSystem system(gfg, *preconditioner_, keyInfo, lambda);
  Vector x0 = initial.vector(keyInfo.ordering());
  const Vector sol = preconditionedConjugateGradient(system, x0, parameters_);

  return buildVectorValues(sol, keyInfo);
}

/*****************************************************************************/
GaussianFactorGraphSystem::GaussianFactorGraphSystem(
    const GaussianFactorGraph &gfg, const Preconditioner &preconditioner,
    const KeyInfo &keyInfo, const std::map<Key, Vector> &lambda) :
    gfg_(gfg), preconditioner_(preconditioner), keyInfo_(keyInfo), lambda_(
        lambda) {
}

/*****************************************************************************/
void GaussianFactorGraphSystem::residual(const Vector &x, Vector &r) const {
  /* implement b-Ax, assume x and r are pre-allocated */

  /* reset r to b */
  getb(r);

  /* substract A*x */
  Vector Ax = Vector::Zero(r.rows(), 1);
  multiply(x, Ax);
  r -= Ax;
}

/*****************************************************************************/
void GaussianFactorGraphSystem::multiply(const Vector &x, Vector& AtAx) const {
  /* implement A^T*(A*x), assume x and AtAx are pre-allocated */

  // Build a VectorValues for Vector x
  VectorValues vvX = buildVectorValues(x, keyInfo_);

  // VectorValues form of A'Ax for multiplyHessianAdd
  VectorValues vvAtAx = keyInfo_.x0(); // crucial for performance

  // vvAtAx += 1.0 * A'Ax for each factor
  gfg_.multiplyHessianAdd(1.0, vvX, vvAtAx);

  // Make the result as Vector form
  AtAx = vvAtAx.vector(keyInfo_.ordering());
}

/*****************************************************************************/
void GaussianFactorGraphSystem::getb(Vector &b) const {
  /* compute rhs, assume b pre-allocated */

  // Get whitened r.h.s (A^T * b) from each factor in the form of VectorValues
  VectorValues vvb = gfg_.gradientAtZero();

  // Make the result as Vector form
  b = -vvb.vector(keyInfo_.ordering());
}

/**********************************************************************************/
void GaussianFactorGraphSystem::leftPrecondition(const Vector &x,
    Vector &y) const {
  // For a preconditioner M = L*L^T
  // Calculate y = L^{-1} x
  preconditioner_.solve(x, y);
}

/**********************************************************************************/
void GaussianFactorGraphSystem::rightPrecondition(const Vector &x,
    Vector &y) const {
  // For a preconditioner M = L*L^T
  // Calculate y = L^{-T} x
  preconditioner_.transposeSolve(x, y);
}

/**********************************************************************************/
void GaussianFactorGraphSystem::scal(const double alpha, Vector &x) const {
  x *= alpha;
}
double GaussianFactorGraphSystem::dot(const Vector &x, const Vector &y) const {
  return x.dot(y);
}
void GaussianFactorGraphSystem::axpy(const double alpha, const Vector &x,
                                     Vector &y) const {
  y += alpha * x;
}
/**********************************************************************************/
VectorValues buildVectorValues(const Vector &v, const Ordering &ordering,
    const map<Key, size_t> & dimensions) {
  VectorValues result;

  DenseIndex offset = 0;
  for (size_t i = 0; i < ordering.size(); ++i) {
    const Key key = ordering[i];
    map<Key, size_t>::const_iterator it = dimensions.find(key);
    if (it == dimensions.end()) {
      throw invalid_argument(
          "buildVectorValues: inconsistent ordering and dimensions");
    }
    const size_t dim = it->second;
    result.emplace(key, v.segment(offset, dim));
    offset += dim;
  }

  return result;
}

/**********************************************************************************/
VectorValues buildVectorValues(const Vector &v, const KeyInfo &keyInfo) {
  VectorValues result;
  for ( const KeyInfo::value_type &item: keyInfo ) {
    result.emplace(item.first, v.segment(item.second.start, item.second.dim));
  }
  return result;
}

}

/*
 * SQPLineSearch2.cpp
 * @brief:
 * @author: Ivan Dario Jimenez
 */

#include <boost/range/adaptor/map.hpp>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/nonlinear/SQPLineSearch2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/nonlinear/MeritFunction.h>

using namespace std;

namespace gtsam {
/* ************************************************************************* */
MeritFunction::MeritFunction(const NP & program,
                             const GaussianFactorGraph::shared_ptr linearizedCost,
                             const GaussianFactorGraph::shared_ptr lagrangianGraph, const Values& x,
                             const VectorValues& p) :
  program_(program), linearizedCost_(linearizedCost), lagrangianGraph_(
  lagrangianGraph), x_(x), p_(p) {
  GTSAM_PRINT(*linearizedCost);
  
//  linearizedCost->error(p) + linearizedCost->gradient()
  gradf_ = linearizedCost_->gradientAtZero();
  std::cout << linearizedCost_->jacobian().first << std::endl;
  GTSAM_PRINT(gradf_);
  GTSAM_PRINT(linearizedCost_->gradient(p));
  if (gradf_.size() < p_.size()) {
    for (Key key : p_ | boost::adaptors::map_keys) {
      if (!gradf_.exists(key)) {
        gradf_.insert(key, zero(p_.at(key).size()));
      }
    }
  }
}

/* ************************************************************************* */
double MeritFunction::constraintNorm1(const Values x) const {
  double norm1 = 0.0;
  for (NonlinearInequalityConstraint::shared_ptr factor : *program_.inequalities) {
    Vector error = factor->unwhitenedError(x);
    norm1 += error.cwiseAbs().sum();
  }
  for (NonlinearConstraint::shared_ptr factor : *program_.equalities) {
    Vector error = factor->unwhitenedError(x);
    norm1 += error.cwiseAbs().sum();
  }
  return norm1;
}

/* ************************************************************************* */
double MeritFunction::phi(double alpha, double mu) const {
  static const bool debug = false;
  Values x2 = (fabs(alpha) > 1e-5) ? x_.retract(alpha * p_) : x_;
  double c2 = constraintNorm1(x2);
  
  double result = program_.cost->error(x2) + mu * c2;
  if (debug)
    cout << "phi(" << alpha << ") = " << result << endl;
  return result;
}

/* ************************************************************************* */
double MeritFunction::D(double mu) const {
  static const bool debug = false;
  double result = p_.dot(gradf_)/linearizedCost_->jacobian().second.lpNorm<1>() - mu * constraintNorm1(x_);
  if (debug)
    cout << "D() = " << result << endl;
  return result;
}

/* ************************************************************************* */
double MeritFunction::computeNewMu(double currentMu) const {
  static const bool debug = false;
  static const double rho = 0.7;
    //gradient * b
    // correct
  double muLowerBound = (p_.dot(gradf_)/linearizedCost_->jacobian().second.lpNorm<1>() + 0.5 * ptHp(lagrangianGraph_, p_))
                                           / ((1 - rho) * constraintNorm1(x_));
  if (debug)
    cout << "gradfk'p = " << (p_.dot(gradf_))/linearizedCost_->jacobian().second.lpNorm<1>() << endl;
  if (debug)
    cout << "ptHp = " << ptHp(lagrangianGraph_, p_) << endl;
  if (debug)
    cout << "||c||_1 = " << constraintNorm1(x_) << endl;
  if (debug)
    cout << "mu lower bound = " << muLowerBound << endl;
  if (currentMu > muLowerBound)
    return currentMu;
  else
    return muLowerBound * 1.1;
}

/* ************************************************************************* */
double MeritFunction::ptHp(const GaussianFactorGraph::shared_ptr linear,
                           const VectorValues& p) const {
  double result = 0.0;
  for (const GaussianFactor::shared_ptr& factor : *linear) {
    VectorValues y = VectorValues::Zero(p);
    factor->multiplyHessianAdd(1.0, p, y); // y = Hx
    result += p.dot(y); // x'y = x'Hx
  }
  return result;
}
}
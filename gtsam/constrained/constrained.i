//*************************************************************************
// constrained
//*************************************************************************
namespace gtsam {

#include <gtsam/constrained/ConstrainedOptProblem.h>
class ConstrainedOptProblem {
  ConstrainedOptProblem();

  std::tuple<double, double, double> evaluate(
      const gtsam::Values& values) const;
  std::tuple<size_t, size_t, size_t> dim() const;
};

#include <gtsam/constrained/LinearConstraint.h>
class LinearConstraint {
  enum class Sense { Equal, LessEqual, GreaterEqual };

  LinearConstraint(const gtsam::JacobianFactor& factor,
                   gtsam::LinearConstraint::Sense sense);
  LinearConstraint(const gtsam::JacobianFactor& factor,
                   gtsam::LinearConstraint::Sense sense,
                   const gtsam::Vector& sigmas);

  static gtsam::LinearConstraint Equal(const gtsam::JacobianFactor& factor);
  static gtsam::LinearConstraint Equal(const gtsam::JacobianFactor& factor,
                                       const gtsam::Vector& sigmas);
  static gtsam::LinearConstraint LessEqual(const gtsam::JacobianFactor& factor);
  static gtsam::LinearConstraint LessEqual(const gtsam::JacobianFactor& factor,
                                           const gtsam::Vector& sigmas);
  static gtsam::LinearConstraint GreaterEqual(
      const gtsam::JacobianFactor& factor);
  static gtsam::LinearConstraint GreaterEqual(
      const gtsam::JacobianFactor& factor, const gtsam::Vector& sigmas);

  gtsam::LinearConstraint::Sense sense() const;
  bool isEquality() const;
  const gtsam::JacobianFactor& factor() const;
  const gtsam::Vector& sigmas() const;
};

#include <gtsam/constrained/QuadraticConstraint.h>
class QuadraticConstraint {
  enum class Sense { Equal, LessEqual, GreaterEqual };

  QuadraticConstraint(gtsam::Key key, const gtsam::Matrix& A, double b,
                      gtsam::QuadraticConstraint::Sense sense);
  QuadraticConstraint(gtsam::Key key, const gtsam::Matrix& A, double b,
                      gtsam::QuadraticConstraint::Sense sense, double sigma);

  static gtsam::QuadraticConstraint Equal(gtsam::Key key,
                                          const gtsam::Matrix& A, double b);
  static gtsam::QuadraticConstraint Equal(gtsam::Key key,
                                          const gtsam::Matrix& A, double b,
                                          double sigma);
  static gtsam::QuadraticConstraint LessEqual(gtsam::Key key,
                                              const gtsam::Matrix& A, double b);
  static gtsam::QuadraticConstraint LessEqual(gtsam::Key key,
                                              const gtsam::Matrix& A, double b,
                                              double sigma);
  static gtsam::QuadraticConstraint GreaterEqual(gtsam::Key key,
                                                 const gtsam::Matrix& A,
                                                 double b);
  static gtsam::QuadraticConstraint GreaterEqual(gtsam::Key key,
                                                 const gtsam::Matrix& A,
                                                 double b, double sigma);

  gtsam::Key key() const;
  const gtsam::Matrix& A() const;
  double b() const;
  gtsam::QuadraticConstraint::Sense sense() const;
  bool isEquality() const;
  double sigma() const;
};

#include <gtsam/constrained/LpProblem.h>
class LpCost {
  LpCost(const gtsam::JacobianFactor& factor);

  const gtsam::JacobianFactor& factor() const;
  double value(const gtsam::Values& values) const;
};

class LpProblem : gtsam::ConstrainedOptProblem {
  LpProblem();

  void addCost(const gtsam::LpCost& cost);
  void addCost(const gtsam::JacobianFactor& factor);
  void addConstraint(const gtsam::LinearConstraint& constraint);

  double objective(const gtsam::Values& values) const;
  gtsam::Values optimize(const gtsam::Values& initialValues) const;
  gtsam::Values optimize() const;
  std::tuple<double, double, double> evaluate(
      const gtsam::Values& values) const;
  std::tuple<size_t, size_t, size_t> dim() const;
};

#include <gtsam/constrained/QpCost.h>
virtual class QpCost : gtsam::NonlinearFactor {
  QpCost();
  QpCost(const gtsam::HessianFactor& factor);
  QpCost(const gtsam::GaussianFactor& factor);
  QpCost(const gtsam::KeyVector& keys, const gtsam::SymmetricBlockMatrix& Q,
         size_t columnDim = 1);

  const gtsam::HessianFactor& hessianFactor() const;
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& formatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& other, double tol = 1e-9) const;
  double error(const gtsam::Values& values) const;
  size_t dim() const;
};

#include <gtsam/constrained/QpProblem.h>
enum class QpSolverType { Sparse, Dense };

class QpProblem : gtsam::ConstrainedOptProblem {
  QpProblem();

  void addCost(const gtsam::QpCost& cost);
  void addCost(const gtsam::HessianFactor& factor);
  void addCost(const gtsam::GaussianFactor& factor);
  void addConstraint(const gtsam::LinearConstraint& constraint);

  gtsam::Values optimize(
      const gtsam::Values& initialValues,
      gtsam::QpSolverType solverType = gtsam::QpSolverType::Sparse) const;
  gtsam::Values optimize(
      gtsam::QpSolverType solverType = gtsam::QpSolverType::Sparse) const;
  std::tuple<double, double, double> evaluate(
      const gtsam::Values& values) const;
  std::tuple<size_t, size_t, size_t> dim() const;
};

#include <gtsam/constrained/QcqpProblem.h>
class QcqpProblem : gtsam::ConstrainedOptProblem {
  QcqpProblem();

  void addCost(const gtsam::QpCost& cost);
  void addConstraint(const gtsam::LinearConstraint& constraint);
  void addConstraint(const gtsam::QuadraticConstraint& constraint);

  std::tuple<double, double, double> evaluate(
      const gtsam::Values& values) const;
  std::tuple<size_t, size_t, size_t> dim() const;
};

#include <gtsam/constrained/ConstrainedOptimizer.h>
class ConstrainedOptimizerParams {
  ConstrainedOptimizerParams();

  size_t maxIterations;
  double absoluteViolationTolerance;
  double relativeViolationTolerance;
  double absoluteCostTolerance;
  double relativeCostTolerance;
  bool verbose;
  bool storeOptProgress;
};

#include <gtsam/constrained/PenaltyOptimizer.h>
class PenaltyOptimizerParams : gtsam::ConstrainedOptimizerParams {
  PenaltyOptimizerParams();

  double initialMuEq;
  double initialMuIneq;
  double muEqIncreaseRate;
  double muIneqIncreaseRate;
};

#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
class AugmentedLagrangianParams : gtsam::PenaltyOptimizerParams {
  AugmentedLagrangianParams();

  double maxDualStepSizeEq;
  double maxDualStepSizeIneq;
  double dualStepSizeFactorEq;
  double dualStepSizeFactorIneq;
  double muIncreaseThreshold;
};

virtual class AugmentedLagrangianOptimizer {
  AugmentedLagrangianOptimizer(
      const gtsam::ConstrainedOptProblem& problem,
      const gtsam::Values& initialValues,
      gtsam::AugmentedLagrangianParams::shared_ptr p);

  gtsam::Values optimize() const;
};

}  // namespace gtsam

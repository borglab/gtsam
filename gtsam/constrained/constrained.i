//*************************************************************************
// constrained
//*************************************************************************
namespace gtsam {

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/SphericalCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/constrained/NonlinearEquality.h>
template <T = {gtsam::Point2,
               gtsam::StereoPoint2,
               gtsam::Point3,
               gtsam::Rot2,
               gtsam::SO3,
               gtsam::SO4,
               gtsam::SOn,
               gtsam::SL4,
               gtsam::Rot3,
               gtsam::Pose2,
               gtsam::Gal3,
               gtsam::Pose3,
               gtsam::Similarity2,
               gtsam::Similarity3,
               gtsam::Cal3_S2,
               gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::SphericalCamera,
               gtsam::imuBias::ConstantBias}>
virtual class NonlinearEquality : gtsam::NoiseModelFactor {
  // Constructor - forces exact evaluation
  NonlinearEquality(gtsam::Key j, const T& feasible);
  // Constructor - allows inexact evaluation
  NonlinearEquality(gtsam::Key j, const T& feasible, double error_gain);

  // enabling serialization functionality
  void serialize() const;
};

template <T = {gtsam::Point2,
               gtsam::StereoPoint2,
               gtsam::Point3,
               gtsam::Rot2,
               gtsam::Gal3,
               gtsam::SO3,
               gtsam::SO4,
               gtsam::SOn,
               gtsam::SL4,
               gtsam::Rot3,
               gtsam::Pose2,
               gtsam::Pose3,
               gtsam::Similarity2,
               gtsam::Similarity3,
               gtsam::Cal3_S2,
               gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::imuBias::ConstantBias}>
virtual class NonlinearEquality2 : gtsam::NoiseModelFactor {
  NonlinearEquality2(gtsam::Key key1, gtsam::Key key2, double mu = 1e4);
  gtsam::Vector evaluateError(const T& x1, const T& x2,
                              gtsam::OptionalMatrixType H1 = nullptr,
                              gtsam::OptionalMatrixType H2 = nullptr) const;
};

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

#include <gtsam/constrained/ActiveSetSolver.h>
class ActiveSetSolverParams {
  enum class QpSubproblemSolver { Sparse, Dense };

  ActiveSetSolverParams();

  size_t maxIterations;
  double activeTolerance;
  double stepTolerance;
  double feasibilityTolerance;
  double multiplierTolerance;
  double regularization;
  double phaseOneFeasibilityTolerance;
  gtsam::ActiveSetSolverParams::QpSubproblemSolver qpSubproblemSolver;
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
  gtsam::Values optimize(
      const gtsam::Values& initialValues,
      std::shared_ptr<gtsam::ActiveSetSolverParams> params = nullptr) const;
  gtsam::Values optimize(
      std::shared_ptr<gtsam::ActiveSetSolverParams> params = nullptr) const;
};

#include <gtsam/constrained/QpCost.h>
virtual class QpCost : gtsam::NonlinearFactor {
  QpCost();
  QpCost(const gtsam::HessianFactor& factor);
  QpCost(const gtsam::GaussianFactor& factor);
  QpCost(const gtsam::KeyVector& keys, const gtsam::SymmetricBlockMatrix& Q,
         size_t columnDim = 1);

  const gtsam::HessianFactor& hessianFactor() const;
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
};

#include <gtsam/constrained/QcqpProblem.h>
class QcqpProblem : gtsam::ConstrainedOptProblem {
  QcqpProblem();
  QcqpProblem(const gtsam::NonlinearFactorGraph& graph,
               size_t columnDimension = 1);

  void addCost(const gtsam::QpCost& cost);
  void addConstraint(const gtsam::LinearConstraint& constraint);
  void addConstraint(const gtsam::QuadraticConstraint& constraint);

};

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>

template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3}>
gtsam::Matrix qcqpValue(const T& typedValue);

template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3}>
void insertQcqpValue(gtsam::Key key, const T& typedValue,
                     gtsam::Values& qcqpValues);

template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3}>
T fromQcqpValue(const gtsam::Matrix& qcqpValue);

template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3}>
gtsam::Values extractQcqpValues(const gtsam::Values& qcqpValues);

#include <gtsam/constrained/ConstrainedOptimizer.h>
class ConstrainedOptimizerParams {
  ConstrainedOptimizerParams();

  // The wrapper generator marshals int as a scalar on every supported target;
  // the underlying C++ field remains size_t.
  int maxIterations;
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
enum class AugmentedLagrangianUpdatePolicy { Aggressive, BCL };

class AugmentedLagrangianParams : gtsam::PenaltyOptimizerParams {
  AugmentedLagrangianParams();

  gtsam::AugmentedLagrangianUpdatePolicy updatePolicy;
  double maxDualStepSizeEq;
  double maxDualStepSizeIneq;
  double dualStepSizeFactorEq;
  double dualStepSizeFactorIneq;
  double muIncreaseThreshold;
  double absoluteStationarityTolerance;
  double bclInitialPenalty;
  double bclPenaltyIncreaseRate;
  double bclOmega0;
  double bclEta0;
  double bclGamma1;
  double bclAlphaOmega;
  double bclBetaOmega;
  double bclAlphaEta;
  double bclBetaEta;
};

virtual class AugmentedLagrangianOptimizer {
  AugmentedLagrangianOptimizer(
      const gtsam::ConstrainedOptProblem& problem,
      const gtsam::Values& initialValues,
      gtsam::AugmentedLagrangianParams::shared_ptr p);

  gtsam::Values optimize() const;
};

}  // namespace gtsam

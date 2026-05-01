/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LeggedEstimator.cpp
 * @date February 2026
 * @author Frank Dellaert
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/LeggedEstimator.h>
#include <gtsam/navigation/LeggedEstimatorFactors.h>
#include <gtsam/navigation/NavStateImuEKF.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <algorithm>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace gtsam {

namespace {

using noiseModel::Diagonal;
using noiseModel::Gaussian;
using noiseModel::Isotropic;
using symbol_shorthand::X;

Key initializationFootKey(size_t foot) {
  return Symbol('f', static_cast<uint64_t>(foot));
}

std::vector<std::string> defaultFootNames(size_t numFeet) {
  std::vector<std::string> names;
  names.reserve(numFeet);
  for (size_t foot = 0; foot < numFeet; ++foot) {
    names.push_back("foot_" + std::to_string(foot));
  }
  return names;
}

void throwIfInvalidParams(const LeggedEstimatorParams& params) {
  if (!params.preintegrationParams) {
    throw std::invalid_argument(
        "LeggedEstimator: preintegrationParams must not be null.");
  }
  if (params.footholdInitSigma <= 0.0) {
    throw std::invalid_argument(
        "LeggedEstimator: footholdInitSigma must be positive.");
  }
  if (!params.contactCovariance.isApprox(params.contactCovariance.transpose(),
                                         1e-12)) {
    throw std::invalid_argument(
        "LeggedEstimator: contactCovariance must be symmetric.");
  }
  if (params.contactCovariance.diagonal().minCoeff() <= 0.0) {
    throw std::invalid_argument(
        "LeggedEstimator: contactCovariance diagonal entries must be "
        "positive.");
  }
  if (params.heightPriorSigma <= 0.0) {
    throw std::invalid_argument(
        "LeggedEstimator: heightPriorSigma must be positive.");
  }
  if (params.useRobustContactNoise && params.robustContactHuberK <= 0.0) {
    throw std::invalid_argument(
        "LeggedEstimator: robustContactHuberK must be positive.");
  }
  if (params.biasAccRandomWalkSigma <= 0.0 ||
      params.biasOmegaRandomWalkSigma <= 0.0) {
    throw std::invalid_argument(
        "LeggedEstimator: bias random-walk sigmas must be positive.");
  }
}

Matrix zeroFootholds(size_t numFeet) {
  Matrix footholds(3, static_cast<Eigen::Index>(numFeet));
  footholds.setZero();
  return footholds;
}

Vector3 unbiasedOmega(const LeggedEstimatorParams& params,
                      const Vector3& omegaBody) {
  return omegaBody - params.imuBias.gyroscope();
}

Vector3 unbiasedSpecificForce(const LeggedEstimatorParams& params,
                              const Vector3& specificForceBody) {
  return specificForceBody - params.imuBias.accelerometer();
}

int footBlockStart(size_t foot) { return 9 + 3 * static_cast<int>(foot); }

Matrix3 contactCovarianceFrom(
    const LeggedEstimatorParams& params,
    const std::optional<Matrix3>& overrideCovariance) {
  if (overrideCovariance) {
    return *overrideCovariance;
  }
  return params.contactCovariance;
}

SharedNoiseModel fixedLagBiasPriorModel() {
  Vector6 sigmas;
  // The contact-event smoother only constrains bias intermittently, so a tight
  // prior keeps the single persistent bias variable from absorbing large
  // translational drift between sparse contact updates.
  sigmas << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
  return Diagonal::Sigmas(sigmas);
}

std::shared_ptr<PreintegrationCombinedParams> combinedPreintegrationParams(
    const LeggedEstimatorParams& params) {
  auto combinedParams = std::dynamic_pointer_cast<PreintegrationCombinedParams>(
      params.preintegrationParams);
  if (!combinedParams) {
    combinedParams = std::make_shared<PreintegrationCombinedParams>(
        params.preintegrationParams->n_gravity);
    combinedParams->gyroscopeCovariance =
        params.preintegrationParams->gyroscopeCovariance;
    combinedParams->accelerometerCovariance =
        params.preintegrationParams->accelerometerCovariance;
    combinedParams->integrationCovariance =
        params.preintegrationParams->integrationCovariance;
    combinedParams->use2ndOrderCoriolis =
        params.preintegrationParams->use2ndOrderCoriolis;
    combinedParams->omegaCoriolis = params.preintegrationParams->omegaCoriolis;
    combinedParams->body_P_sensor = params.preintegrationParams->body_P_sensor;
  }
  combinedParams->biasAccCovariance =
      I_3x3 * (params.biasAccRandomWalkSigma * params.biasAccRandomWalkSigma);
  combinedParams->biasOmegaCovariance =
      I_3x3 *
      (params.biasOmegaRandomWalkSigma * params.biasOmegaRandomWalkSigma);
  return combinedParams;
}

LevenbergMarquardtParams leggedLmParams() {
  LevenbergMarquardtParams params;
  params.lambdaInitial = 1e3;
  params.maxIterations = 50;
  return params;
}

Point3 imuMeasurement(const Pose3& body_P_imu, const Vector3& bodyPoint);

}  // namespace

namespace {

SharedNoiseModel fullContactInitializationBasePrior() {
  Vector9 sigmas;
  // Only roll, pitch, and base height should move appreciably during the
  // contact-based inverse-kinematics solve. Planar translation, yaw, and
  // velocity are treated as fixed gauge choices for initialization.
  sigmas << 1.0, 1.0, 1e-6, 1e-6, 1e-6, 1.0, 1e-6, 1e-6, 1e-6;
  return Diagonal::Sigmas(sigmas);
}

void validateContactPacket(
    const std::vector<ContactMeasurement>& activeContacts, size_t numFeet,
    const char* context) {
  std::set<size_t> seen;
  for (const ContactMeasurement& contact : activeContacts) {
    if (contact.foot >= numFeet) {
      throw std::out_of_range(std::string(context) +
                              ": foot index out of range.");
    }
    if (!seen.insert(contact.foot).second) {
      throw std::invalid_argument(std::string(context) +
                                  ": duplicate foot measurement.");
    }
  }
}

void replaceMarginalizedFootCovariance(Matrix& covariance, size_t foot,
                                       double sigma) {
  const int dim = static_cast<int>(covariance.rows());
  const int start = footBlockStart(foot);
  std::vector<int> retainedIndices;
  retainedIndices.reserve(dim - 3);
  for (int index = 0; index < dim; ++index) {
    if (index < start || index >= start + 3) {
      retainedIndices.push_back(index);
    }
  }

  Matrix retainedCovariance(dim - 3, dim - 3);
  for (int row = 0; row < dim - 3; ++row) {
    for (int col = 0; col < dim - 3; ++col) {
      retainedCovariance(row, col) =
          covariance(retainedIndices[row], retainedIndices[col]);
    }
  }

  Matrix replacedCovariance = Matrix::Zero(dim, dim);
  for (int row = 0; row < dim - 3; ++row) {
    for (int col = 0; col < dim - 3; ++col) {
      replacedCovariance(retainedIndices[row], retainedIndices[col]) =
          retainedCovariance(row, col);
    }
  }
  replacedCovariance.block(start, start, 3, 3) = I_3x3 * (sigma * sigma);
  covariance = replacedCovariance;
}

Point3 imuMeasurement(const Pose3& body_P_imu, const Vector3& bodyPoint) {
  return body_P_imu.transformTo(Point3(bodyPoint));
}

Point3 footholdFromMeasurement(const Pose3& body_P_imu, const NavState& state,
                               const Vector3& bodyPoint) {
  return state.pose().transformFrom(imuMeasurement(body_P_imu, bodyPoint));
}

SharedNoiseModel fullCovarianceModel(const Matrix& covariance) {
  return Gaussian::Covariance(covariance);
}

SharedNoiseModel robustContactNoiseModel(const LeggedEstimatorParams& params,
                                         const Matrix3& covariance) {
  if (!params.useRobustContactNoise) {
    return fullCovarianceModel(covariance);
  }
  return noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(
          params.robustContactHuberK, noiseModel::mEstimator::Base::Scalar),
      fullCovarianceModel(covariance));
}

struct FullContactInitializationResult {
  NavState navState;
  Matrix footholds;
  Matrix covarianceWorldFoot;
};

struct InitializedExtendedPosePosterior {
  ExtendedPose3d state;
  Matrix covariance;
};

Matrix3 fullContactFootCovariance(const Matrix& covarianceWorldFoot,
                                  size_t foot) {
  return covarianceWorldFoot.block<3, 3>(footBlockStart(foot),
                                         footBlockStart(foot));
}

Matrix regularizedCovariance(const Matrix& covariance, double minVariance) {
  const Matrix symmetric = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    return symmetric +
           Matrix::Identity(covariance.rows(), covariance.cols()) * minVariance;
  }
  Vector eigenvalues = solver.eigenvalues();
  eigenvalues = eigenvalues.array().max(minVariance);
  return solver.eigenvectors() * eigenvalues.asDiagonal() *
         solver.eigenvectors().transpose();
}

Matrix initialSmootherCovariance(const Matrix9& baseCovariance, size_t numFeet,
                                 double footholdSigma) {
  const int dim = 9 + 3 * static_cast<int>(numFeet);
  Matrix covariance = Matrix::Zero(dim, dim);
  covariance.topLeftCorner<9, 9>() = baseCovariance;
  for (size_t foot = 0; foot < numFeet; ++foot) {
    covariance.block(footBlockStart(foot), footBlockStart(foot), 3, 3) =
        I_3x3 * (footholdSigma * footholdSigma);
  }
  return covariance;
}

NavState fullContactInitializationNavState(
    const NavState& seedNavState,
    const std::vector<ContactMeasurement>& activeContacts,
    const LeggedEstimatorParams& params, double terrainHeight) {
  Matrix imuContacts(3, static_cast<Eigen::Index>(activeContacts.size()));
  for (size_t index = 0; index < activeContacts.size(); ++index) {
    imuContacts.col(static_cast<Eigen::Index>(index)) =
        imuMeasurement(params.body_P_imu, activeContacts[index].bodyPoint);
  }

  const Vector3 centroid = imuContacts.rowwise().mean();
  const Matrix centered = imuContacts.colwise() - centroid;
  Vector3 normal =
      Eigen::JacobiSVD<Matrix>(centered, Eigen::ComputeFullU).matrixU().col(2);
  if (normal.z() < 0.0) {
    normal = -normal;
  }

  const double roll = std::atan2(normal.y(), normal.z());
  const double pitch = std::asin(-normal.x());
  const Vector3 seedRpy = seedNavState.attitude().rpy();
  const Rot3 attitude = Rot3::Ypr(seedRpy.z(), pitch, roll);

  double height = 0.0;
  for (const ContactMeasurement& contact : activeContacts) {
    const Point3 measurement =
        imuMeasurement(params.body_P_imu, contact.bodyPoint);
    height += terrainHeight - attitude.matrix().row(2).dot(measurement);
  }
  height /= static_cast<double>(activeContacts.size());

  const Point3 position(seedNavState.position().x(),
                        seedNavState.position().y(), height);
  return NavState(attitude, position, Vector3::Zero());
}

KeyVector fullContactInitializationKeys(size_t numFeet) {
  KeyVector keys;
  keys.reserve(1 + numFeet);
  keys.push_back(X(0));
  for (size_t foot = 0; foot < numFeet; ++foot) {
    keys.push_back(initializationFootKey(foot));
  }
  return keys;
}

FullContactInitializationResult solveFullContactInitialization(
    const NavState& seedNavState,
    const std::vector<ContactMeasurement>& activeContacts,
    const LeggedEstimatorParams& params, size_t numFeet, double terrainHeight) {
  NonlinearFactorGraph graph;
  Values values;
  const Key baseKey = X(0);
  graph.emplace_shared<PriorFactor<NavState>>(
      baseKey, seedNavState, fullContactInitializationBasePrior());
  const NavState initializedNavState = fullContactInitializationNavState(
      seedNavState, activeContacts, params, terrainHeight);
  values.insert(baseKey, initializedNavState);

  const SharedNoiseModel contactNoise =
      fullCovarianceModel(contactCovarianceFrom(params, std::nullopt));
  const SharedNoiseModel heightNoise =
      Isotropic::Sigma(1, params.heightPriorSigma);
  for (const ContactMeasurement& contact : activeContacts) {
    const Key footKey = initializationFootKey(contact.foot);
    const Point3 foothold = footholdFromMeasurement(
        params.body_P_imu, initializedNavState, contact.bodyPoint);
    values.insert(footKey, foothold);
    graph.emplace_shared<NavStatePointContactFactor>(
        baseKey, footKey, imuMeasurement(params.body_P_imu, contact.bodyPoint),
        contactNoise);
    graph.emplace_shared<PointHeightFactor>(footKey, terrainHeight,
                                            heightNoise);
  }

  Matrix footholds = zeroFootholds(numFeet);
  for (size_t foot = 0; foot < numFeet; ++foot) {
    const auto contactIt =
        std::find_if(activeContacts.begin(), activeContacts.end(),
                     [foot](const ContactMeasurement& measurement) {
                       return measurement.foot == foot;
                     });
    if (contactIt == activeContacts.end()) {
      footholds.col(static_cast<Eigen::Index>(foot)).setZero();
      continue;
    }
    footholds.col(static_cast<Eigen::Index>(foot)) = footholdFromMeasurement(
        params.body_P_imu, initializedNavState, contactIt->bodyPoint);
  }

  const KeyVector keys = fullContactInitializationKeys(numFeet);
  Values gaugeFixedValues = values;
  for (size_t foot = 0; foot < numFeet; ++foot) {
    gaugeFixedValues.update(
        initializationFootKey(foot),
        Point3(footholds.col(static_cast<Eigen::Index>(foot))));
  }
  Marginals marginals(graph, gaugeFixedValues);

  return {initializedNavState, footholds,
          marginals.jointMarginalCovariance(keys).fullMatrix()};
}

class IkInitializer {
 public:
  IkInitializer(const LeggedEstimatorParams& params, size_t numFeet,
                double terrainHeight)
      : params_(params), numFeet_(numFeet), terrainHeight_(terrainHeight) {}

  InitializedExtendedPosePosterior fuse(
      const ExtendedPose3d& priorState, const Matrix& priorCovariance,
      const std::vector<ContactMeasurement>& activeContacts) const {
    const NavState seedNavState(priorState.rotation(), priorState.x(0),
                                priorState.x(1));
    const FullContactInitializationResult initialization =
        solveFullContactInitialization(seedNavState, activeContacts, params_,
                                       numFeet_, terrainHeight_);
    Matrix blocks(
        3, static_cast<Eigen::Index>(2 + initialization.footholds.cols()));
    blocks.col(0) = initialization.navState.position();
    blocks.col(1) = initialization.navState.velocity();
    blocks.rightCols(initialization.footholds.cols()) =
        initialization.footholds;

    NonlinearFactorGraph graph;
    Values values;
    const Key key = X(0);
    graph.emplace_shared<PriorFactor<ExtendedPose3d>>(key, priorState,
                                                      priorCovariance);
    values.insert(key,
                  ExtendedPose3d(initialization.navState.attitude(), blocks));

    const SharedNoiseModel contactNoise =
        robustContactNoiseModel(params_, params_.contactCovariance);
    const SharedNoiseModel heightNoise =
        Isotropic::Sigma(1, params_.heightPriorSigma);
    for (const ContactMeasurement& contact : activeContacts) {
      graph.emplace_shared<ExtendedPoseContactFactor>(
          key, LeggedInvariantEKF::FootColumn(contact.foot),
          imuMeasurement(params_.body_P_imu, contact.bodyPoint), contactNoise);
      graph.emplace_shared<ExtendedPoseHeightFactor>(
          key, LeggedInvariantEKF::FootColumn(contact.foot), terrainHeight_,
          heightNoise);
    }

    LevenbergMarquardtOptimizer optimizer(graph, values, leggedLmParams());
    const Values result = optimizer.optimize();
    Marginals marginals(graph, result);
    return {result.at<ExtendedPose3d>(key), marginals.marginalCovariance(key)};
  }

 private:
  const LeggedEstimatorParams& params_;
  size_t numFeet_;
  double terrainHeight_;
};

}  // namespace

/* ************************************************************************* */
ExtendedPose3d LeggedEstimator::MakeEstimate(const NavState& navState,
                                             const Matrix& footholds) {
  Matrix blocks(3, static_cast<Eigen::Index>(2 + footholds.cols()));
  blocks.col(0) = navState.position();
  blocks.col(1) = navState.velocity();
  blocks.rightCols(footholds.cols()) = footholds;
  return ExtendedPose3d(navState.attitude(), blocks);
}

/* ************************************************************************* */
Matrix LeggedEstimator::EstimateFootholds(const ExtendedPose3d& estimate) {
  if (estimate.k() < 2) {
    throw std::invalid_argument(
        "LeggedEstimator::EstimateFootholds: estimate must contain position "
        "and velocity blocks.");
  }
  const Eigen::Index numFeet =
      static_cast<Eigen::Index>(estimate.k() - static_cast<size_t>(2));
  return estimate.xMatrix().rightCols(numFeet);
}

/* ************************************************************************* */
LeggedInvariantEKF::LeggedInvariantEKF(
    const NavState& navState0, const Matrix& footholds0, const Matrix& P0,
    const LeggedEstimatorParams& params,
    const std::vector<std::string>& footNames)
    : EkfBase(MakeState(navState0, footholds0), P0),
      numFeet_(static_cast<size_t>(footholds0.cols())),
      params_(params),
      footNames_(footNames.empty() ? defaultFootNames(numFeet_) : footNames),
      inContact_(numFeet_, false),
      initialized_(numFeet_, false) {
  throwIfInvalidParams(params_);
  if (footNames_.size() != numFeet_) {
    throw std::invalid_argument(
        "LeggedInvariantEKF: footNames must match the number of feet.");
  }
  if (P0.rows() != static_cast<int>(MakeState(navState0, footholds0).dim()) ||
      P0.cols() != static_cast<int>(MakeState(navState0, footholds0).dim())) {
    throw std::invalid_argument(
        "LeggedInvariantEKF: covariance dimension does not match state.");
  }
  initialized_.assign(numFeet_, true);
}

/* ************************************************************************* */
void LeggedInvariantEKF::processContacts(
    const std::vector<ContactMeasurement>& activeContacts) {
  validateContactPacket(activeContacts, numFeet_,
                        "LeggedInvariantEKF::processContacts");

  std::vector<ContactMeasurement> sortedContacts = activeContacts;
  std::sort(sortedContacts.begin(), sortedContacts.end(),
            [](const ContactMeasurement& a, const ContactMeasurement& b) {
              return a.foot < b.foot;
            });

  std::vector<bool> activeFeet(numFeet_, false);
  for (const ContactMeasurement& contact : sortedContacts) {
    activeFeet[contact.foot] = true;
  }

  if (awaitingFullContactInitialization()) {
    (void)maybeInitializeFromFullContact(sortedContacts, activeFeet);
    return;
  }

  for (size_t foot = 0; foot < numFeet_; ++foot) {
    if (!inContact_[foot] || activeFeet[foot]) {
      continue;
    }
    if (params_.marginalizeLeavingFoot) {
      marginalizeFoot(foot);
      initialized_[foot] = false;
    }
    inContact_[foot] = false;
  }

  for (const ContactMeasurement& contact : sortedContacts) {
    if (!contact.touchdown && inContact_[contact.foot] &&
        initialized_[contact.foot]) {
      continue;
    }
    resetFootToMeasurement(contact.foot, contact.bodyPoint);
    initialized_[contact.foot] = true;
  }
  applyContactUpdate(sortedContacts);
  inContact_ = activeFeet;
}

/* ************************************************************************* */
bool LeggedInvariantEKF::maybeInitializeFromFullContact(
    const std::vector<ContactMeasurement>& activeContacts,
    const std::vector<bool>& activeFeet) {
  if (!params_.useFullContactInitialization || fullContactInitialized_ ||
      numFeet_ < 3 || activeContacts.size() != numFeet_) {
    return false;
  }

  const double terrainHeightValue = terrainHeight().value_or(0.0);
  const InitializedExtendedPosePosterior initialization =
      IkInitializer(params_, numFeet_, terrainHeightValue)
          .fuse(this->X_, this->P_, activeContacts);
  this->X_ = initialization.state;
  this->P_ = initialization.covariance;
  inContact_ = activeFeet;
  std::fill(initialized_.begin(), initialized_.end(), true);
  fullContactInitialized_ = true;
  return true;
}

/* ************************************************************************* */
ExtendedPose3d LeggedInvariantEKF::MakeState(const NavState& navState,
                                             const Matrix& footholds) {
  return MakeEstimate(navState, footholds);
}

/* ************************************************************************* */
ExtendedPose3d LeggedInvariantEKF::GravityIncrement(size_t numFeet,
                                                    const Vector3& gravity,
                                                    double dt) {
  Matrix blocks = zeroFootholds(2 + numFeet);
  blocks.col(0) = gravity * (0.5 * dt * dt);
  blocks.col(1) = gravity * dt;
  return ExtendedPose3d(Rot3(), blocks);
}

/* ************************************************************************* */
ExtendedPose3d LeggedInvariantEKF::ImuIncrement(
    size_t numFeet, const Vector3& omegaBody, const Vector3& specificForceBody,
    double dt) {
  const Vector3 phiBody = omegaBody * dt;
  const so3::DexpFunctor local(phiBody);
  Matrix blocks = zeroFootholds(2 + numFeet);
  blocks.col(0) = local.Gamma().left() * specificForceBody * dt * dt;
  blocks.col(1) = local.Jacobian().left() * specificForceBody * dt;
  return ExtendedPose3d(Rot3::Expmap(phiBody), blocks);
}

/* ************************************************************************* */
LeggedInvariantEKF::Covariance LeggedInvariantEKF::processNoise(
    double dt) const {
  const int dim = 9 + 3 * static_cast<int>(numFeet());
  Covariance Q = Covariance::Zero(dim, dim);
  const auto& pim = *params().preintegrationParams;
  Q.block(0, 0, 3, 3) = pim.gyroscopeCovariance * dt;
  Q.block(3, 3, 3, 3) = pim.integrationCovariance * dt;
  Q.block(6, 6, 3, 3) = pim.accelerometerCovariance * dt;

  const Matrix3 footNoise = I_3x3 * (params().footholdProcessSigma *
                                     params().footholdProcessSigma * dt);
  for (size_t foot = 0; foot < numFeet(); ++foot) {
    Q.block(footBlockStart(foot), footBlockStart(foot), 3, 3) = footNoise;
  }
  return Q;
}

/* ************************************************************************* */
void LeggedInvariantEKF::predict(const Vector3& omegaBody,
                                 const Vector3& specificForceBody, double dt) {
  if (dt <= 0.0) {
    throw std::invalid_argument(
        "LeggedInvariantEKF::predict: dt must be positive.");
  }
  if (awaitingFullContactInitialization()) {
    return;
  }

  const Vector3 correctedOmegaBody = unbiasedOmega(params(), omegaBody);
  const Vector3 correctedSpecificForceBody =
      unbiasedSpecificForce(params(), specificForceBody);
  const ExtendedPose3d W =
      GravityIncrement(numFeet(), params().preintegrationParams->n_gravity, dt);
  const ExtendedPose3d U = ImuIncrement(numFeet(), correctedOmegaBody,
                                        correctedSpecificForceBody, dt);
  const AutonomousFlow phi(numFeet(), dt);
  EkfBase::predict(W, phi, U, processNoise(dt));
}

/* ************************************************************************* */
void LeggedInvariantEKF::resetFootToMeasurement(size_t foot,
                                                const Vector3& bodyPoint) {
  Matrix blocks = this->X_.xMatrix();
  blocks.col(static_cast<Eigen::Index>(FootColumn(foot))) =
      footholdFromMeasurement(params().body_P_imu, baseState(), bodyPoint);
  this->X_ = ExtendedPose3d(this->X_.rotation(), blocks);
  replaceMarginalizedFootCovariance(this->P_, foot, params().footholdInitSigma);
}

/* ************************************************************************* */
void LeggedInvariantEKF::marginalizeFoot(size_t foot) {
  Matrix blocks = this->X_.xMatrix();
  blocks.col(static_cast<Eigen::Index>(FootColumn(foot))).setZero();
  this->X_ = ExtendedPose3d(this->X_.rotation(), blocks);
  replaceMarginalizedFootCovariance(this->P_, foot, params().footholdInitSigma);
}

void LeggedInvariantEKF::applySingleContactUpdate(size_t foot,
                                                  const Vector3& bodyPoint,
                                                  const Matrix3& covariance) {
  const Point3 z = imuMeasurement(params().body_P_imu, bodyPoint);
  Matrix measurement_H_state;
  const Point3 prediction = extendedPoseContactPrediction(
      this->X_, FootColumn(foot), &measurement_H_state);
  EkfBase::update<Point3>(prediction, measurement_H_state, z, covariance);
}

/* ************************************************************************* */
void LeggedInvariantEKF::applySingleHeightPrior(size_t foot,
                                                double terrainHeight) {
  Matrix prior_H_state =
      Matrix::Zero(1, static_cast<Eigen::Index>(this->X_.dim()));
  prior_H_state.block(0, footBlockStart(foot), 1, 3) =
      this->X_.rotation().matrix().row(2);
  EkfBase::update<Vector>(
      Vector1(this->X_.x(FootColumn(foot)).z()), prior_H_state,
      Vector1(terrainHeight),
      I_1x1 * (params().heightPriorSigma * params().heightPriorSigma));
}

/* ************************************************************************* */
void LeggedInvariantEKF::applyContactUpdate(
    const std::vector<ContactMeasurement>& activeContacts) {
  const Matrix3 covariance = params().contactCovariance;
  for (const ContactMeasurement& contact : activeContacts) {
    if (terrainHeight()) {
      applySingleHeightPrior(contact.foot, *terrainHeight());
    }
    applySingleContactUpdate(contact.foot, contact.bodyPoint, covariance);
  }
}

LeggedInvariantIEKF::LeggedInvariantIEKF(
    const NavState& navState0, const Matrix& footholds0, const Matrix& P0,
    const LeggedEstimatorParams& params,
    const std::vector<std::string>& footNames)
    : LeggedInvariantEKF(navState0, footholds0, P0, params, footNames) {}

/* ************************************************************************* */
void LeggedInvariantIEKF::applyContactUpdate(
    const std::vector<ContactMeasurement>& activeContacts) {
  NonlinearFactorGraph graph;
  Values values;
  const Key key = X(0);

  graph.emplace_shared<PriorFactor<ExtendedPose3d>>(key, this->X_, this->P_);
  values.insert(key, this->X_);

  const SharedNoiseModel contactNoise =
      robustContactNoiseModel(params(), params().contactCovariance);
  for (const ContactMeasurement& contact : activeContacts) {
    graph.emplace_shared<ExtendedPoseContactFactor>(
        key, FootColumn(contact.foot),
        imuMeasurement(params().body_P_imu, contact.bodyPoint), contactNoise);
    if (terrainHeight()) {
      graph.emplace_shared<ExtendedPoseHeightFactor>(
          key, FootColumn(contact.foot), *terrainHeight(),
          Isotropic::Sigma(1, params().heightPriorSigma));
    }
  }

  const LevenbergMarquardtParams optimizerParams = leggedLmParams();
  LevenbergMarquardtOptimizer optimizer(graph, values, optimizerParams);
  const Values result = optimizer.optimize();
  Marginals marginals(graph, result);
  this->X_ = result.at<ExtendedPose3d>(key);
  this->P_ = marginals.marginalCovariance(key);
}

/* ************************************************************************* */
LeggedFixedLagSmoother::LeggedFixedLagSmoother(
    const NavState& navState0, const Matrix& footholds0,
    const Matrix9& baseCovariance0, const LeggedEstimatorParams& params,
    double lagSeconds, const std::vector<std::string>& footNames)
    : numFeet_(static_cast<size_t>(footholds0.cols())),
      params_(params),
      footNames_(footNames.empty() ? defaultFootNames(numFeet_) : footNames),
      initialFootholds_(footholds0),
      baseCovariance0_(baseCovariance0),
      smoother_(lagSeconds, leggedLmParams()),
      pim_(params.preintegrationParams, imuBias::ConstantBias()),
      inContact_(numFeet_, false),
      initialized_(numFeet_, false),
      footEpisodes_(numFeet_, 0),
      activeFootKeys_(numFeet_),
      optimizedBaseState_(navState0),
      deadReckonedState_(navState0) {
  throwIfInvalidParams(params_);
  if (footNames_.size() != numFeet_) {
    throw std::invalid_argument(
        "LeggedFixedLagSmoother: footNames must match the number of feet.");
  }

  if (params_.useFullContactInitialization) {
    return;
  }

  // Start the smoother with a single prior on the initial base state.
  NonlinearFactorGraph factors;
  Values values;
  FixedLagSmoother::KeyTimestampMap timestamps;
  const Key baseKey = MakeBaseKey(0);
  const Key biasKey = MakeBiasKey();
  factors.emplace_shared<PriorFactor<NavState>>(baseKey, navState0,
                                                baseCovariance0);
  factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(
      biasKey, biasEstimate_, fixedLagBiasPriorModel());
  values.insert(baseKey, navState0);
  values.insert(biasKey, biasEstimate_);
  timestamps[baseKey] = 0.0;
  timestamps[biasKey] = 0.0;
  smoother_.update(factors, values, timestamps);
  refreshEstimateFromSmoother();
}

/* ************************************************************************* */
LeggedFixedLagSmoother::~LeggedFixedLagSmoother() = default;

/* ************************************************************************* */
ExtendedPose3d LeggedFixedLagSmoother::estimate() const {
  if (!graphInitialized()) {
    return MakeEstimate(deadReckonedState_, initialFootholds_);
  }
  const Values values = smoother_.calculateEstimate();
  Matrix footholds = zeroFootholds(numFeet_);
  for (size_t foot = 0; foot < numFeet_; ++foot) {
    if (activeFootKeys_[foot] && values.exists(*activeFootKeys_[foot])) {
      footholds.col(static_cast<Eigen::Index>(foot)) =
          values.at<Point3>(*activeFootKeys_[foot]);
    }
  }
  return MakeEstimate(deadReckonedState_, footholds);
}

/* ************************************************************************* */
void LeggedFixedLagSmoother::predict(const Vector3& omegaBody,
                                     const Vector3& specificForceBody,
                                     double dt) {
  if (dt <= 0.0) {
    throw std::invalid_argument(
        "LeggedFixedLagSmoother::predict: dt must be positive.");
  }
  if (awaitingFullContactInitialization()) {
    return;
  }

  // PreintegratedImuMeasurements expects raw IMU samples and applies the
  // current bias linearization point internally.
  pim_.integrateMeasurement(specificForceBody, omegaBody, dt);
  currentTime_ += dt;
  deadReckonedState_ = pim_.predict(optimizedBaseState_, biasEstimate_);
}

/* ************************************************************************* */
void LeggedFixedLagSmoother::processContacts(
    const std::vector<ContactMeasurement>& activeContacts) {
  validateContactPacket(activeContacts, numFeet_,
                        "LeggedFixedLagSmoother::processContacts");

  // Sort contacts once so graph construction and state bookkeeping are
  // deterministic.
  std::vector<ContactMeasurement> sortedContacts = activeContacts;
  std::sort(sortedContacts.begin(), sortedContacts.end(),
            [](const ContactMeasurement& a, const ContactMeasurement& b) {
              return a.foot < b.foot;
            });

  // Mark which foot indices are active in this packet.
  std::vector<bool> activeFeet(numFeet_, false);
  for (const ContactMeasurement& contact : sortedContacts) {
    activeFeet[contact.foot] = true;
  }

  if (awaitingFullContactInitialization()) {
    (void)maybeInitializeFromFullContact(sortedContacts, activeFeet);
    return;
  }

  for (size_t foot = 0; foot < numFeet_; ++foot) {
    // Dropped contacts end the current episode and release its active key.
    if (inContact_[foot] && !activeFeet[foot]) {
      inContact_[foot] = false;
      initialized_[foot] = false;
      activeFootKeys_[foot].reset();
    }
  }

  if (sortedContacts.empty()) {
    // Swing-only packets update bookkeeping but do not create smoother nodes.
    return;
  }

  // Assemble all contact and terrain factors that apply at the current base
  // time.
  NonlinearFactorGraph factors;
  Values values;
  FixedLagSmoother::KeyTimestampMap timestamps;
  timestamps[MakeBiasKey()] = currentTime_;

  Key baseKey = currentBaseKey();
  NavState baseState = currentBaseState();
  if (hasPendingImu()) {
    // Close the accumulated IMU interval with a new base node at this contact
    // event.
    const Key previousBaseKey = baseKey;
    ++step_;
    baseKey = currentBaseKey();
    baseState = deadReckonedState_;
    factors.emplace_shared<ImuFactor2>(previousBaseKey, baseKey, MakeBiasKey(),
                                       pim_);
    values.insert(baseKey, baseState);
    timestamps[baseKey] = currentTime_;
  } else {
    // Multiple contact packets at the same timestamp refine the current base
    // node.
    timestamps[baseKey] = currentTime_;
  }

  const SharedNoiseModel contactNoise =
      robustContactNoiseModel(params_, params_.contactCovariance);

  for (const ContactMeasurement& contact : sortedContacts) {
    if (contact.touchdown || !activeFootKeys_[contact.foot]) {
      // A new touchdown starts a fresh landmark episode with its own smoother
      // key.
      ++footEpisodes_[contact.foot];
      const Key footKey =
          MakeFootKey(contact.foot, footEpisodes_[contact.foot]);
      activeFootKeys_[contact.foot] = footKey;
      initialized_[contact.foot] = true;
      const Point3 foothold = footholdFromMeasurement(
          params_.body_P_imu, baseState, contact.bodyPoint);
      values.insert(footKey, foothold);
      // Touchdown reinitialization in the smoother gets an explicit point prior
      // so the new landmark episode carries the same uncertainty as the filter
      // variants after replacing a foot block.
      factors.emplace_shared<PriorFactor<Point3>>(
          footKey, foothold, Isotropic::Sigma(3, params_.footholdInitSigma));
    }

    // Tie the current base state to the active foot episode through a contact
    // factor.
    const Key footKey = *activeFootKeys_[contact.foot];
    factors.emplace_shared<NavStatePointContactFactor>(
        baseKey, footKey, imuMeasurement(params_.body_P_imu, contact.bodyPoint),
        contactNoise);
    if (terrainHeight()) {
      // Terrain height is modeled as an additional unary prior on the foot
      // landmark.
      factors.emplace_shared<PointHeightFactor>(
          footKey, *terrainHeight(),
          Isotropic::Sigma(1, params_.heightPriorSigma));
    }
    // Refresh the foot timestamp so active contact episodes stay inside the lag
    // window.
    timestamps[footKey] = currentTime_;
    inContact_[contact.foot] = true;
  }

  smoother_.update(factors, values, timestamps);
  refreshEstimateFromSmoother();
  // Rebase dead reckoning on the optimized event state and bias estimate.
  pim_.resetIntegrationAndSetBias(biasEstimate_);
  deadReckonedState_ = optimizedBaseState_;
}

/* ************************************************************************* */
void LeggedFixedLagSmoother::refreshEstimateFromSmoother() {
  if (!graphInitialized()) {
    return;
  }
  const Values values = smoother_.calculateEstimate();
  if (values.exists(currentBaseKey())) {
    // Pull the latest base estimate from the smoother if the current key
    // survives the lag.
    optimizedBaseState_ = values.at<NavState>(currentBaseKey());
  }
  if (values.exists(MakeBiasKey())) {
    // Keep the preintegration bias linearization point synchronized with the
    // graph.
    biasEstimate_ = values.at<imuBias::ConstantBias>(MakeBiasKey());
  }
}

/* ************************************************************************* */
bool LeggedFixedLagSmoother::maybeInitializeFromFullContact(
    const std::vector<ContactMeasurement>& activeContacts,
    const std::vector<bool>& activeFeet) {
  if (!params_.useFullContactInitialization || fullContactInitialized_ ||
      numFeet_ < 3 || activeContacts.size() != numFeet_) {
    return false;
  }

  const double terrainHeightValue = terrainHeight().value_or(0.0);
  const InitializedExtendedPosePosterior initialization =
      IkInitializer(params_, numFeet_, terrainHeightValue)
          .fuse(MakeEstimate(deadReckonedState_, initialFootholds_),
                initialSmootherCovariance(baseCovariance0_, numFeet_,
                                          params_.footholdInitSigma),
                activeContacts);

  smoother_ =
      BatchFixedLagSmoother(smoother_.smootherLag(), smoother_.params());
  pim_.resetIntegrationAndSetBias(imuBias::ConstantBias());
  step_ = 0;
  currentTime_ = 0.0;
  footEpisodes_.assign(numFeet_, 0);
  activeFootKeys_.assign(numFeet_, std::nullopt);
  inContact_.assign(numFeet_, false);
  initialized_.assign(numFeet_, false);

  NonlinearFactorGraph factors;
  Values values;
  FixedLagSmoother::KeyTimestampMap timestamps;
  const Key baseKey = MakeBaseKey(0);
  const Key biasKey = MakeBiasKey();
  const NavState initializedBaseState = EstimateNavState(initialization.state);
  const Matrix initializedFootholds = EstimateFootholds(initialization.state);
  values.insert(baseKey, initializedBaseState);
  values.insert(biasKey, biasEstimate_);
  factors.emplace_shared<PriorFactor<NavState>>(
      baseKey, initializedBaseState,
      regularizedCovariance(initialization.covariance.topLeftCorner<9, 9>(),
                            1e-6));
  factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(
      biasKey, biasEstimate_, fixedLagBiasPriorModel());
  timestamps[biasKey] = 0.0;
  timestamps[baseKey] = 0.0;

  for (const ContactMeasurement& contact : activeContacts) {
    ++footEpisodes_[contact.foot];
    const Key footKey = MakeFootKey(contact.foot, footEpisodes_[contact.foot]);
    activeFootKeys_[contact.foot] = footKey;
    const Point3 foothold =
        initializedFootholds.col(static_cast<Eigen::Index>(contact.foot));
    values.insert(footKey, foothold);
    factors.emplace_shared<PriorFactor<Point3>>(
        footKey, foothold,
        Gaussian::Covariance(regularizedCovariance(
            fullContactFootCovariance(initialization.covariance, contact.foot),
            1e-6)));
    inContact_[contact.foot] = activeFeet[contact.foot];
    initialized_[contact.foot] = true;
    timestamps[footKey] = 0.0;
  }

  smoother_.update(factors, values, timestamps);
  fullContactInitialized_ = true;
  refreshEstimateFromSmoother();
  pim_.resetIntegrationAndSetBias(biasEstimate_);
  deadReckonedState_ = optimizedBaseState_;
  return true;
}

/* ************************************************************************* */
LeggedCombinedFixedLagSmoother::LeggedCombinedFixedLagSmoother(
    const NavState& navState0, const Matrix& footholds0,
    const Matrix9& baseCovariance0, const LeggedEstimatorParams& params,
    double lagSeconds, const std::vector<std::string>& footNames)
    : numFeet_(static_cast<size_t>(footholds0.cols())),
      params_(params),
      footNames_(footNames.empty() ? defaultFootNames(numFeet_) : footNames),
      initialFootholds_(footholds0),
      baseCovariance0_(baseCovariance0),
      smoother_(lagSeconds, leggedLmParams()),
      pim_(combinedPreintegrationParams(params), params.imuBias),
      inContact_(numFeet_, false),
      initialized_(numFeet_, false),
      footEpisodes_(numFeet_, 0),
      activeFootKeys_(numFeet_),
      optimizedBaseState_(navState0),
      deadReckonedState_(navState0),
      biasEstimate_(params.imuBias) {
  throwIfInvalidParams(params_);
  if (footNames_.size() != numFeet_) {
    throw std::invalid_argument(
        "LeggedCombinedFixedLagSmoother: footNames must match the number of "
        "feet.");
  }

  if (params_.useFullContactInitialization) {
    return;
  }

  // Start the smoother with priors on the initial pose, velocity, and bias.
  NonlinearFactorGraph factors;
  Values values;
  FixedLagSmoother::KeyTimestampMap timestamps;
  const Key poseKey = MakePoseKey(0);
  const Key velocityKey = MakeVelocityKey(0);
  const Key biasKey = MakeBiasKey(0);
  factors.emplace_shared<PriorFactor<Pose3>>(
      poseKey, navState0.pose(), baseCovariance0.topLeftCorner<6, 6>());
  factors.emplace_shared<PriorFactor<Vector3>>(
      velocityKey, navState0.velocity(),
      baseCovariance0.bottomRightCorner<3, 3>());
  factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(
      biasKey, biasEstimate_, fixedLagBiasPriorModel());
  values.insert(poseKey, navState0.pose());
  values.insert(velocityKey, navState0.velocity());
  values.insert(biasKey, biasEstimate_);
  timestamps[poseKey] = 0.0;
  timestamps[velocityKey] = 0.0;
  timestamps[biasKey] = 0.0;
  smoother_.update(factors, values, timestamps);
  refreshEstimateFromSmoother();
}

/* ************************************************************************* */
LeggedCombinedFixedLagSmoother::~LeggedCombinedFixedLagSmoother() = default;

/* ************************************************************************* */
ExtendedPose3d LeggedCombinedFixedLagSmoother::estimate() const {
  if (!graphInitialized()) {
    return MakeEstimate(deadReckonedState_, initialFootholds_);
  }
  const Values values = smoother_.calculateEstimate();
  Matrix footholds = zeroFootholds(numFeet_);
  for (size_t foot = 0; foot < numFeet_; ++foot) {
    if (activeFootKeys_[foot] && values.exists(*activeFootKeys_[foot])) {
      footholds.col(static_cast<Eigen::Index>(foot)) =
          values.at<Point3>(*activeFootKeys_[foot]);
    }
  }
  return MakeEstimate(deadReckonedState_, footholds);
}

/* ************************************************************************* */
void LeggedCombinedFixedLagSmoother::predict(const Vector3& omegaBody,
                                             const Vector3& specificForceBody,
                                             double dt) {
  if (dt <= 0.0) {
    throw std::invalid_argument(
        "LeggedCombinedFixedLagSmoother::predict: dt must be positive.");
  }
  if (awaitingFullContactInitialization()) {
    return;
  }

  // Combined preintegration consumes raw IMU and applies the current bias hat.
  pim_.integrateMeasurement(specificForceBody, omegaBody, dt);
  currentTime_ += dt;
  deadReckonedState_ = pim_.predict(optimizedBaseState_, biasEstimate_);
}

/* ************************************************************************* */
void LeggedCombinedFixedLagSmoother::processContacts(
    const std::vector<ContactMeasurement>& activeContacts) {
  validateContactPacket(activeContacts, numFeet_,
                        "LeggedCombinedFixedLagSmoother::processContacts");

  // Sort contacts once so graph construction and state bookkeeping are
  // deterministic.
  std::vector<ContactMeasurement> sortedContacts = activeContacts;
  std::sort(sortedContacts.begin(), sortedContacts.end(),
            [](const ContactMeasurement& a, const ContactMeasurement& b) {
              return a.foot < b.foot;
            });

  // Mark which foot indices are active in this packet.
  std::vector<bool> activeFeet(numFeet_, false);
  for (const ContactMeasurement& contact : sortedContacts) {
    activeFeet[contact.foot] = true;
  }

  if (awaitingFullContactInitialization()) {
    (void)maybeInitializeFromFullContact(sortedContacts, activeFeet);
    return;
  }

  for (size_t foot = 0; foot < numFeet_; ++foot) {
    // Dropped contacts end the current episode and release its active key.
    if (inContact_[foot] && !activeFeet[foot]) {
      inContact_[foot] = false;
      initialized_[foot] = false;
      activeFootKeys_[foot].reset();
    }
  }

  if (sortedContacts.empty()) {
    // Swing-only packets update bookkeeping but do not create smoother nodes.
    return;
  }

  // Assemble all contact and terrain factors that apply at the current base
  // time.
  NonlinearFactorGraph factors;
  Values values;
  FixedLagSmoother::KeyTimestampMap timestamps;

  Key poseKey = currentPoseKey();
  Key velocityKey = currentVelocityKey();
  Key biasKey = currentBiasKey();
  NavState baseState = currentBaseState();
  if (hasPendingImu()) {
    // Close the accumulated IMU interval with a new state and bias at this
    // contact event.
    const Key previousPoseKey = poseKey;
    const Key previousVelocityKey = velocityKey;
    const Key previousBiasKey = biasKey;
    ++step_;
    poseKey = currentPoseKey();
    velocityKey = currentVelocityKey();
    biasKey = currentBiasKey();
    baseState = deadReckonedState_;
    factors.emplace_shared<CombinedImuFactor>(
        previousPoseKey, previousVelocityKey, poseKey, velocityKey,
        previousBiasKey, biasKey, pim_);
    values.insert(poseKey, baseState.pose());
    values.insert(velocityKey, baseState.velocity());
    values.insert(biasKey, biasEstimate_);
  }
  timestamps[poseKey] = currentTime_;
  timestamps[velocityKey] = currentTime_;
  timestamps[biasKey] = currentTime_;

  const SharedNoiseModel contactNoise =
      robustContactNoiseModel(params_, params_.contactCovariance);

  for (const ContactMeasurement& contact : sortedContacts) {
    if (contact.touchdown || !activeFootKeys_[contact.foot]) {
      // A new touchdown starts a fresh landmark episode with its own smoother
      // key.
      ++footEpisodes_[contact.foot];
      const Key footKey =
          MakeFootKey(contact.foot, footEpisodes_[contact.foot]);
      activeFootKeys_[contact.foot] = footKey;
      initialized_[contact.foot] = true;
      const Point3 foothold = footholdFromMeasurement(
          params_.body_P_imu, baseState, contact.bodyPoint);
      values.insert(footKey, foothold);
      // Touchdown reinitialization uses the same loose point prior as the
      // filter variants' foot-block replacement.
      factors.emplace_shared<PriorFactor<Point3>>(
          footKey, foothold, Isotropic::Sigma(3, params_.footholdInitSigma));
    }

    // Tie the current pose estimate to the active foot episode through a
    // contact factor.
    const Key footKey = *activeFootKeys_[contact.foot];
    factors.emplace_shared<Pose3PointContactFactor>(
        poseKey, footKey, imuMeasurement(params_.body_P_imu, contact.bodyPoint),
        contactNoise);
    if (terrainHeight()) {
      // Terrain height is modeled as an additional unary prior on the foot
      // landmark.
      factors.emplace_shared<PointHeightFactor>(
          footKey, *terrainHeight(),
          Isotropic::Sigma(1, params_.heightPriorSigma));
    }
    // Refresh the foot timestamp so active contact episodes stay inside the lag
    // window.
    timestamps[footKey] = currentTime_;
    inContact_[contact.foot] = true;
  }

  smoother_.update(factors, values, timestamps);
  refreshEstimateFromSmoother();
  // Rebase dead reckoning on the optimized event state and bias estimate.
  pim_.resetIntegrationAndSetBias(biasEstimate_);
  deadReckonedState_ = optimizedBaseState_;
}

/* ************************************************************************* */
void LeggedCombinedFixedLagSmoother::refreshEstimateFromSmoother() {
  if (!graphInitialized()) {
    return;
  }
  const Values values = smoother_.calculateEstimate();
  if (values.exists(currentPoseKey()) && values.exists(currentVelocityKey())) {
    // Pull the latest base estimate from the smoother if the current keys
    // survive the lag.
    optimizedBaseState_ = NavState(values.at<Pose3>(currentPoseKey()),
                                   values.at<Vector3>(currentVelocityKey()));
  }
  if (values.exists(currentBiasKey())) {
    // Keep the preintegration bias linearization point synchronized with the
    // graph.
    biasEstimate_ = values.at<imuBias::ConstantBias>(currentBiasKey());
  }
}

/* ************************************************************************* */
bool LeggedCombinedFixedLagSmoother::maybeInitializeFromFullContact(
    const std::vector<ContactMeasurement>& activeContacts,
    const std::vector<bool>& activeFeet) {
  if (!params_.useFullContactInitialization || fullContactInitialized_ ||
      numFeet_ < 3 || activeContacts.size() != numFeet_) {
    return false;
  }

  const double terrainHeightValue = terrainHeight().value_or(0.0);
  const InitializedExtendedPosePosterior initialization =
      IkInitializer(params_, numFeet_, terrainHeightValue)
          .fuse(MakeEstimate(deadReckonedState_, initialFootholds_),
                initialSmootherCovariance(baseCovariance0_, numFeet_,
                                          params_.footholdInitSigma),
                activeContacts);

  smoother_ =
      BatchFixedLagSmoother(smoother_.smootherLag(), smoother_.params());
  pim_.resetIntegrationAndSetBias(params_.imuBias);
  step_ = 0;
  currentTime_ = 0.0;
  footEpisodes_.assign(numFeet_, 0);
  activeFootKeys_.assign(numFeet_, std::nullopt);
  inContact_.assign(numFeet_, false);
  initialized_.assign(numFeet_, false);

  NonlinearFactorGraph factors;
  Values values;
  FixedLagSmoother::KeyTimestampMap timestamps;
  const Key poseKey = MakePoseKey(0);
  const Key velocityKey = MakeVelocityKey(0);
  const Key biasKey = MakeBiasKey(0);
  const NavState initializedBaseState = EstimateNavState(initialization.state);
  const Matrix initializedFootholds = EstimateFootholds(initialization.state);
  values.insert(poseKey, initializedBaseState.pose());
  values.insert(velocityKey, initializedBaseState.velocity());
  values.insert(biasKey, biasEstimate_);
  factors.emplace_shared<PriorFactor<Pose3>>(
      poseKey, initializedBaseState.pose(),
      regularizedCovariance(initialization.covariance.topLeftCorner<6, 6>(),
                            1e-6));
  factors.emplace_shared<PriorFactor<Vector3>>(
      velocityKey, initializedBaseState.velocity(),
      regularizedCovariance(initialization.covariance.block<3, 3>(6, 6), 1e-6));
  factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(
      biasKey, biasEstimate_, fixedLagBiasPriorModel());
  timestamps[poseKey] = 0.0;
  timestamps[velocityKey] = 0.0;
  timestamps[biasKey] = 0.0;

  for (const ContactMeasurement& contact : activeContacts) {
    ++footEpisodes_[contact.foot];
    const Key footKey = MakeFootKey(contact.foot, footEpisodes_[contact.foot]);
    activeFootKeys_[contact.foot] = footKey;
    const Point3 foothold =
        initializedFootholds.col(static_cast<Eigen::Index>(contact.foot));
    values.insert(footKey, foothold);
    factors.emplace_shared<PriorFactor<Point3>>(
        footKey, foothold,
        Gaussian::Covariance(regularizedCovariance(
            fullContactFootCovariance(initialization.covariance, contact.foot),
            1e-6)));
    inContact_[contact.foot] = activeFeet[contact.foot];
    initialized_[contact.foot] = true;
    timestamps[footKey] = 0.0;
  }

  smoother_.update(factors, values, timestamps);
  fullContactInitialized_ = true;
  refreshEstimateFromSmoother();
  pim_.resetIntegrationAndSetBias(biasEstimate_);
  deadReckonedState_ = optimizedBaseState_;
  return true;
}

}  // namespace gtsam

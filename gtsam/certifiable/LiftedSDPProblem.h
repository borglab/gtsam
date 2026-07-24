#pragma once

#include <gtsam/config.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/symbolic/SymbolicBayesTree.h>

#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace gtsam {

struct MonolithicSDP {};

struct ChordalSDP {};

enum class ChordalOrderingType { Metis, Colamd };

struct MosekSDPSolver {};

template <typename SDPFormulation, typename SDPSolver>
class LiftedSDPProblem;

#ifdef GTSAM_USE_MOSEK
template <>
class LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> {
 public:

  explicit LiftedSDPProblem(const QcqpProblem& problem);

  ~LiftedSDPProblem();

  // # copied from gtsam-private
  bool solve(const std::map<std::string, double>& mosek_params = {});

  double objectiveValue() const;

  std::string problemStatus() const;

  double solveTimeSeconds() const;

  void recoverLiftedVectors();

  const std::vector<Vector>& getRecoveredLiftedVectors() const;

  const std::vector<double>& getRecoveredVariableEVRs() const;

  // # copied from gtsam-private
  template <typename T>
  std::vector<T> getRecoveredPoses() const {
    const auto& liftedVectors = getRecoveredLiftedVectors();
    std::vector<T> recoveredPoses;
    recoveredPoses.reserve(liftedVectors.size());
    for (const Vector& x : liftedVectors) {
      if constexpr (std::is_same_v<T, Rot2>) {
        if (x.size() != 5 || std::abs(x(0)) < 1e-9) {
          throw std::runtime_error(
              "getRecoveredPoses<Rot2>: invalid lifted vector.");
        }

        const Vector xCanonical = x / x(0);
        Matrix2 Rraw;
        // QcqpValue<Rot2, 1> stores vec(R) in column-major order.
        Rraw << xCanonical(1), xCanonical(3), xCanonical(2), xCanonical(4);
        recoveredPoses.push_back(Rot2::ClosestTo(Rraw));
      } else {
        static_assert(std::is_same_v<T, Rot2>,
                      "No lifted-vector decoder exists for this type.");
      }
    }
    return recoveredPoses;
  }

  // # copied from gtsam-private
  template <typename T>
  std::vector<double> getRecoveredPoseErrorNorms(
      const std::vector<T>& groundTruth) const {
    const std::vector<T> recoveredPoses = getRecoveredPoses<T>();
    if (groundTruth.size() != recoveredPoses.size()) {
      throw std::runtime_error(
          "getRecoveredPoseErrorNorms: ground-truth size does not match key "
          "count.");
    }

    std::vector<double> errors(recoveredPoses.size());
    for (size_t index = 0; index < recoveredPoses.size(); ++index) {
      errors[index] =
          groundTruth[index].localCoordinates(recoveredPoses[index]).norm();
    }
    return errors;
  }

  const KeyVector& orderedKeys() const;

  const std::map<Key, DenseIndex>& orderedKeyDims() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

template <>
class LiftedSDPProblem<ChordalSDP, MosekSDPSolver> {
 public:
  LiftedSDPProblem(const QcqpProblem& problem,
                   ChordalOrderingType orderingType);

  ~LiftedSDPProblem();

  // # modified from gtsam-private
  bool solve(const std::map<std::string, double>& mosek_params = {});

  double objectiveValue() const;

  std::string problemStatus() const;

  double solveTimeSeconds() const;

  void recoverLiftedVectors();

  const std::vector<Vector>& getRecoveredLiftedVectors() const;

  const std::vector<double>& getRecoveredVariableEVRs() const;

  // # modified from gtsam-private
  template <typename T>
  std::vector<T> getRecoveredPoses() const {
    const auto& liftedVectors = getRecoveredLiftedVectors();
    std::vector<T> recoveredPoses;
    recoveredPoses.reserve(liftedVectors.size());
    for (const Vector& x : liftedVectors) {
      if constexpr (std::is_same_v<T, Rot2>) {
        if (x.size() != 5 || std::abs(x(0)) < 1e-9) {
          throw std::runtime_error(
              "getRecoveredPoses<Rot2>: invalid lifted vector.");
        }

        const Vector xCanonical = x / x(0);
        Matrix2 Rraw;
        // QcqpValue<Rot2, 1> stores vec(R) in column-major order.
        Rraw << xCanonical(1), xCanonical(3), xCanonical(2), xCanonical(4);
        recoveredPoses.push_back(Rot2::ClosestTo(Rraw));
      } else {
        static_assert(std::is_same_v<T, Rot2>,
                      "No lifted-vector decoder exists for this type.");
      }
    }
    return recoveredPoses;
  }

  // # modified from gtsam-private
  template <typename T>
  std::vector<double> getRecoveredPoseErrorNorms(
      const std::vector<T>& groundTruth) const {
    const std::vector<T> recoveredPoses = getRecoveredPoses<T>();
    if (groundTruth.size() != recoveredPoses.size()) {
      throw std::runtime_error(
          "getRecoveredPoseErrorNorms: ground-truth size does not match key "
          "count.");
    }

    std::vector<double> errors(recoveredPoses.size());
    for (size_t index = 0; index < recoveredPoses.size(); ++index) {
      errors[index] =
          groundTruth[index].localCoordinates(recoveredPoses[index]).norm();
    }
    return errors;
  }

  const KeyVector& orderedKeys() const;

  const std::map<Key, DenseIndex>& orderedKeyDims() const;

  const SymbolicBayesTree& bayesTree() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
#endif

}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file RangeISAMExample_plaza2.cpp
 * @brief A 2D Range SLAM example
 * @date June 20, 2013
 * @author Frank Dellaert
 */
#pragma once
// Both relative poses and recovered trajectory poses will be stored as Pose2.
#include <gtsam/geometry/Pose3.h>
using gtsam::Pose2;

// gtsam::Vectors are dynamic Eigen vectors, Vector3 is statically sized.
#include <gtsam/base/Vector.h>
using gtsam::Vector;
using gtsam::Vector2;
using gtsam::Vector3;

// Unknown landmarks are of type Point2 (which is just a 2D Eigen vector).
#include <gtsam/geometry/Point2.h>
using gtsam::Point2;

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols.
#include <gtsam/inference/Symbol.h>
using gtsam::Symbol;

// We want to use iSAM2 to solve the range-SLAM problem incrementally.
#include <gtsam/nonlinear/ISAM2.h>

// Measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics SLAM problems:
#include <gtsam/slam/WNOAFactors.h>

using gtsam::tictoc_print_;

#include <gtsam/slam/StereoFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

// Standard headers, added last, so we know headers above work on their own.
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <functional>
#include <algorithm>

using namespace gtsam;

template <typename PoseType>
class Interpolator {
protected:
  static constexpr int dim = traits<PoseType>::dimension;
  using VelocityType = typename gtsam::traits<PoseType>::TangentVector;
  using MatrixN = Eigen::Matrix<double, dim, dim>;
  using VectorN = Eigen::Matrix<double, dim, 1>;
  using Matrix2N = Eigen::Matrix<double, 2*dim, 2*dim>;
  using Vector2N = Eigen::Matrix<double, 2*dim, 1>;
  using MatrixNx2N = Eigen::Matrix<double, dim, 2*dim>;

  MatrixN Q_psd_;  // Power Spectral Density for WNOA
  std::function<Matrix(double dt)> transitionFunction_;
  std::function<Matrix(double dt, const MatrixN& Q_psd)> covarianceFunction_;


public:
  Interpolator() = delete;

  Interpolator(const MatrixN& Q_psd,
               std::function<Matrix(double dt)> transitionFunction,
               std::function<Matrix(double dt, const MatrixN& Q_psd)> covarianceFunction)
      : Q_psd_(Q_psd),
        transitionFunction_(transitionFunction),
        covarianceFunction_(covarianceFunction) {}
  
  // Default to WNOA
  Interpolator(const MatrixN& Q_psd)
      : Interpolator(
          Q_psd,
          WNOAMotionFactor<PoseType>::transitionFunction,
          WNOAMotionFactor<PoseType>::buildWNOACovariance) {}

  std::pair<PoseType, VelocityType> interpolatePoseAndVelocity(std::pair<PoseType, VelocityType> Tvarpi_k,
                                      double t_k, std::pair<PoseType, VelocityType> Tvarpi_kp1,
                                      double t_kp1, double t_tau) {

    // if t_tau is equal to t_k or t_kp1, return the corresponding pose and velocity
    if (equal(t_tau, t_k)) {
      return Tvarpi_k;
    } else if (equal(t_tau, t_kp1)) {
      return Tvarpi_kp1;
    } else {
      assert(t_tau >= t_k && t_tau <= t_kp1 && "Query time must be between t_k and t_kp1");
    }

    auto [T_k, varpi_k] = Tvarpi_k;
    auto [T_kp1, varpi_kp1] = Tvarpi_kp1;

    Matrix2N Lambda, Psi;  // ensure Lambda and Psi are 2*dim x 2*dim matrices, rather than using auto below
    std::tie(Lambda, Psi) = getLamdaPsi(t_k, t_kp1, t_tau);

    MatrixNx2N Lambda_1 = Lambda.topRows(dim);
    MatrixNx2N Lambda_2 = Lambda.bottomRows(dim);
    MatrixNx2N Psi_1 = Psi.topRows(dim);
    MatrixNx2N Psi_2 = Psi.bottomRows(dim);
    
    // form quantities for Eq. (11.45) in the book
    Vector2N gamma_k;
    gamma_k.topRows(dim).setZero();
    gamma_k.bottomRows(dim) = varpi_k;
    VectorN xi = traits<PoseType>::Local(T_k, T_kp1);
    MatrixN right_jac = PoseType::ExpmapDerivative(xi);
    Vector2N gamma_kp1;
    gamma_kp1 << xi, right_jac.inverse() * varpi_kp1;

    // Eq. (11.45) in the book
    auto T_tau = T_k.compose(PoseType::Expmap(Lambda_1 * gamma_k + Psi_1 * gamma_kp1));
    auto right_jac_tau = PoseType::ExpmapDerivative(traits<PoseType>::Local(T_k, T_tau));
    auto varpi_tau = right_jac_tau.inverse() * (Lambda_2 * gamma_k + Psi_2 * gamma_kp1);

    return std::make_pair(T_tau, varpi_tau);
  }

  std::vector<std::pair<PoseType, VelocityType>> interpolatePosesAndVelocities(
    std::vector<std::pair<PoseType, VelocityType>> posesAndVelocities,
    std::vector<double> timestamps,
    std::vector<double> queryTimes) {

    assert(std::is_sorted(timestamps.begin(), timestamps.end()) &&
           "Solution timestamps must be sorted in ascending order.");
    std::vector<std::pair<PoseType, VelocityType>> results;
    results.reserve(queryTimes.size());

    for (size_t i = 0; i < queryTimes.size(); ++i) {
        double t_tau = queryTimes[i];
        // Find the interval [t_k, t_kp1] that contains t_tau
        size_t k = 0;
        while (k < timestamps.size() - 1 && !(timestamps[k] <= t_tau && t_tau <= timestamps[k + 1])) {
            ++k;
        }
        if (t_tau < timestamps[0] || t_tau > timestamps[timestamps.size() - 1]) {
            std::cout << "Query time " << t_tau << " is out of bounds of the provided timestamps ranging from " 
                      << timestamps[0] << " to " << timestamps[timestamps.size() - 1] << std::endl;
            throw std::out_of_range("Query time is out of bounds of the provided timestamps.");
        }
        auto [T_tau, varpi_tau] = interpolatePoseAndVelocity(posesAndVelocities[k], timestamps[k], posesAndVelocities[k+1], timestamps[k+1], t_tau);
        results.emplace_back(T_tau, varpi_tau);
    }
    return results;
  }

protected:
  std::pair<Matrix, Matrix> getLamdaPsi(double t_k, double t_kp1, double t_tau) {

    // Build transition matrices for all combinations
    double dt = t_kp1 - t_k;
    auto Phi_12 = transitionFunction_(dt);
    auto Phi_1tau = transitionFunction_(t_tau - t_k);
    auto Phi_tau2 = transitionFunction_(t_kp1 - t_tau);

    // Construct Q
    auto Q_12 = covarianceFunction_(dt, Q_psd_);
    auto Q_1tau = covarianceFunction_(t_tau - t_k, Q_psd_);

    // Eq. (11.41) in the book
    auto Lambda = Phi_1tau - Q_1tau * Phi_tau2.transpose() * Q_12.inverse() * Phi_12;
    auto Psi = Q_1tau * Phi_tau2.transpose() * Q_12.inverse();

    return std::make_pair(Lambda, Psi);
  }

};

// Explicit Instantiation.
template class Interpolator<Pose2>;
template class Interpolator<Pose3>;

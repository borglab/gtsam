/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartFactorParams.h
 * @brief  Collect common parameters for SmartProjection and SmartStereoProjection factors
 * @author Luca Carlone
 * @author Zsolt Kira
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/triangulation.h>

namespace gtsam {

/**
 * SmartFactorParams: parameters and (linearization/degeneracy) modes for SmartProjection and SmartStereoProjection factors
 */
/// Linearization mode: what factor to linearize to
enum LinearizationMode {
  HESSIAN, IMPLICIT_SCHUR, JACOBIAN_Q, JACOBIAN_SVD
};

/// How to manage degeneracy
enum DegeneracyMode {
  IGNORE_DEGENERACY, ZERO_ON_DEGENERACY, HANDLE_INFINITY
};

/*
 *  Parameters for the smart (stereo) projection factors
 */
struct SmartProjectionParams {

  LinearizationMode linearizationMode; ///< How to linearize the factor
  DegeneracyMode degeneracyMode; ///< How to linearize the factor

  /// @name Parameters governing the triangulation
  /// @{
  TriangulationParameters triangulation;
  double retriangulationThreshold; ///< threshold to decide whether to re-triangulate
  /// @}

  /// @name Parameters governing how triangulation result is treated
  /// @{
  bool throwCheirality; ///< If true, re-throws Cheirality exceptions (default: false)
  bool verboseCheirality; ///< If true, prints text for Cheirality exceptions (default: false)
  /// @}

  // Constructor
  SmartProjectionParams(LinearizationMode linMode = HESSIAN,
      DegeneracyMode degMode = IGNORE_DEGENERACY, bool throwCheirality = false,
      bool verboseCheirality = false, double retriangulationTh = 1e-5) :
        linearizationMode(linMode), degeneracyMode(degMode), retriangulationThreshold(
            retriangulationTh), throwCheirality(throwCheirality), verboseCheirality(
                verboseCheirality) {
  }

  virtual ~SmartProjectionParams() {
  }

  void print(const std::string& str = "") const {
    std::cout << "linearizationMode: " << linearizationMode << "\n";
    std::cout << "   degeneracyMode: " << degeneracyMode << "\n";
    std::cout << triangulation << std::endl;
  }

  // get class variables
  LinearizationMode getLinearizationMode() const {
    return linearizationMode;
  }
  DegeneracyMode getDegeneracyMode() const {
    return degeneracyMode;
  }
  TriangulationParameters getTriangulationParameters() const {
    return triangulation;
  }
  bool getVerboseCheirality() const {
    return verboseCheirality;
  }
  bool getThrowCheirality() const {
    return throwCheirality;
  }
  double getRetriangulationThreshold() const {
    return retriangulationThreshold;
  }
  // set class variables
  void setLinearizationMode(LinearizationMode linMode) {
    linearizationMode = linMode;
  }
  void setDegeneracyMode(DegeneracyMode degMode) {
    degeneracyMode = degMode;
  }
  void setRetriangulationThreshold(double retriangulationTh) {
    retriangulationThreshold = retriangulationTh;
  }
  void setRankTolerance(double rankTol) {
    triangulation.rankTolerance = rankTol;
  }
  void setEnableEPI(bool enableEPI) {
    triangulation.enableEPI = enableEPI;
  }
  void setLandmarkDistanceThreshold(double landmarkDistanceThreshold) {
    triangulation.landmarkDistanceThreshold = landmarkDistanceThreshold;
  }
  void setDynamicOutlierRejectionThreshold(double dynOutRejectionThreshold) {
    triangulation.dynamicOutlierRejectionThreshold = dynOutRejectionThreshold;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(linearizationMode);
    ar & BOOST_SERIALIZATION_NVP(degeneracyMode);
    ar & BOOST_SERIALIZATION_NVP(triangulation);
    ar & BOOST_SERIALIZATION_NVP(retriangulationThreshold);
    ar & BOOST_SERIALIZATION_NVP(throwCheirality);
    ar & BOOST_SERIALIZATION_NVP(verboseCheirality);
  }
};

} // \ namespace gtsam

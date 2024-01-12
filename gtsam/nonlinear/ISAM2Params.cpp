/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2Params.cpp
 * @brief   Parameters for iSAM 2.
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

#include <gtsam/nonlinear/ISAM2Params.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
string ISAM2DoglegParams::adaptationModeTranslator(
    const DoglegOptimizerImpl::TrustRegionAdaptationMode& adaptationMode)
    const {
  string s;
  switch (adaptationMode) {
    case DoglegOptimizerImpl::SEARCH_EACH_ITERATION:
      s = "SEARCH_EACH_ITERATION";
      break;
    case DoglegOptimizerImpl::ONE_STEP_PER_ITERATION:
      s = "ONE_STEP_PER_ITERATION";
      break;
    default:
      s = "UNDEFINED";
      break;
  }
  return s;
}

/* ************************************************************************* */
DoglegOptimizerImpl::TrustRegionAdaptationMode
ISAM2DoglegParams::adaptationModeTranslator(
    const string& adaptationMode) const {
  string s = adaptationMode;
  // convert to upper case
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
  if (s == "SEARCH_EACH_ITERATION")
    return DoglegOptimizerImpl::SEARCH_EACH_ITERATION;
  if (s == "ONE_STEP_PER_ITERATION")
    return DoglegOptimizerImpl::ONE_STEP_PER_ITERATION;

  /* default is SEARCH_EACH_ITERATION */
  return DoglegOptimizerImpl::SEARCH_EACH_ITERATION;
}

/* ************************************************************************* */
ISAM2Params::Factorization ISAM2Params::factorizationTranslator(
    const string& str) {
  string s = str;
  // convert to upper case
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
  if (s == "QR") return ISAM2Params::QR;
  if (s == "CHOLESKY") return ISAM2Params::CHOLESKY;

  /* default is CHOLESKY */
  return ISAM2Params::CHOLESKY;
}

/* ************************************************************************* */
string ISAM2Params::factorizationTranslator(
    const ISAM2Params::Factorization& value) {
  string s;
  switch (value) {
    case ISAM2Params::QR:
      s = "QR";
      break;
    case ISAM2Params::CHOLESKY:
      s = "CHOLESKY";
      break;
    default:
      s = "UNDEFINED";
      break;
  }
  return s;
}

}  // namespace gtsam

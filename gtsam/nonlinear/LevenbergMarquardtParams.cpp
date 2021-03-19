/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LevenbergMarquardtParams.cpp
 * @brief   Parameters for Levenberg-Marquardt trust-region scheme
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    Feb 26, 2012
 */

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/range/adaptor/map.hpp>
#include <iostream>
#include <string>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
LevenbergMarquardtParams::VerbosityLM LevenbergMarquardtParams::verbosityLMTranslator(
    const std::string &src) {
  std::string s = src;
  boost::algorithm::to_upper(s);
  if (s == "SILENT")
    return LevenbergMarquardtParams::SILENT;
  if (s == "SUMMARY")
    return LevenbergMarquardtParams::SUMMARY;
  if (s == "TERMINATION")
    return LevenbergMarquardtParams::TERMINATION;
  if (s == "LAMBDA")
    return LevenbergMarquardtParams::LAMBDA;
  if (s == "TRYLAMBDA")
    return LevenbergMarquardtParams::TRYLAMBDA;
  if (s == "TRYCONFIG")
    return LevenbergMarquardtParams::TRYCONFIG;
  if (s == "TRYDELTA")
    return LevenbergMarquardtParams::TRYDELTA;
  if (s == "DAMPED")
    return LevenbergMarquardtParams::DAMPED;

  /* default is silent */
  return LevenbergMarquardtParams::SILENT;
}

/* ************************************************************************* */
std::string LevenbergMarquardtParams::verbosityLMTranslator(
    VerbosityLM value) {
  std::string s;
  switch (value) {
  case LevenbergMarquardtParams::SILENT:
    s = "SILENT";
    break;
  case LevenbergMarquardtParams::SUMMARY:
    s = "SUMMARY";
    break;
  case LevenbergMarquardtParams::TERMINATION:
    s = "TERMINATION";
    break;
  case LevenbergMarquardtParams::LAMBDA:
    s = "LAMBDA";
    break;
  case LevenbergMarquardtParams::TRYLAMBDA:
    s = "TRYLAMBDA";
    break;
  case LevenbergMarquardtParams::TRYCONFIG:
    s = "TRYCONFIG";
    break;
  case LevenbergMarquardtParams::TRYDELTA:
    s = "TRYDELTA";
    break;
  case LevenbergMarquardtParams::DAMPED:
    s = "DAMPED";
    break;
  default:
    s = "UNDEFINED";
    break;
  }
  return s;
}

/* ************************************************************************* */
void LevenbergMarquardtParams::print(const std::string& str) const {
  NonlinearOptimizerParams::print(str);
  std::cout << "              lambdaInitial: " << lambdaInitial << "\n";
  std::cout << "               lambdaFactor: " << lambdaFactor << "\n";
  std::cout << "           lambdaUpperBound: " << lambdaUpperBound << "\n";
  std::cout << "           lambdaLowerBound: " << lambdaLowerBound << "\n";
  std::cout << "           minModelFidelity: " << minModelFidelity << "\n";
  std::cout << "            diagonalDamping: " << diagonalDamping << "\n";
  std::cout << "                minDiagonal: " << minDiagonal << "\n";
  std::cout << "                maxDiagonal: " << maxDiagonal << "\n";
  std::cout << "                verbosityLM: "
      << verbosityLMTranslator(verbosityLM) << "\n";
  std::cout.flush();
}

} /* namespace gtsam */


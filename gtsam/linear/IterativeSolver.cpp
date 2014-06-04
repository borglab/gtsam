/**
 * @file   IterativeSolver.cpp
 * @brief  
 * @date   Sep 3, 2012
 * @author Yong-Dian Jian
 */

#include <gtsam/linear/IterativeSolver.h>
#include <boost/algorithm/string.hpp>
#include <string>

namespace gtsam {

/*****************************************************************************/
std::string IterativeOptimizationParameters::getVerbosity() const { return verbosityTranslator(verbosity_); }

/*****************************************************************************/
void IterativeOptimizationParameters::setVerbosity(const std::string &src) { verbosity_ = verbosityTranslator(src); }

/*****************************************************************************/
void IterativeOptimizationParameters::print() const {
  std::cout << "IterativeOptimizationParameters" << std::endl
            << "verbosity:     " << verbosityTranslator(verbosity_) << std::endl;
}

/*****************************************************************************/
IterativeOptimizationParameters::Verbosity IterativeOptimizationParameters::verbosityTranslator(const std::string &src)  {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "SILENT") return IterativeOptimizationParameters::SILENT;
  else if (s == "COMPLEXITY") return IterativeOptimizationParameters::COMPLEXITY;
  else if (s == "ERROR") return IterativeOptimizationParameters::ERROR;
  /* default is default */
  else return IterativeOptimizationParameters::SILENT;
}

/*****************************************************************************/
std::string IterativeOptimizationParameters::verbosityTranslator(IterativeOptimizationParameters::Verbosity verbosity)  {
  if (verbosity == SILENT) return "SILENT";
  else if (verbosity == COMPLEXITY) return "COMPLEXITY";
  else if (verbosity == ERROR) return "ERROR";
  else return "UNKNOWN";
}


}



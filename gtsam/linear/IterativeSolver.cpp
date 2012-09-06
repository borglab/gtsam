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

std::string IterativeOptimizationParameters::getKernel() const { return kernelTranslator(kernel_); }
std::string IterativeOptimizationParameters::getVerbosity() const { return verbosityTranslator(verbosity_); }
void IterativeOptimizationParameters::setKernel(const std::string &src) { kernel_ = kernelTranslator(src); }
void IterativeOptimizationParameters::setVerbosity(const std::string &src) { verbosity_ = verbosityTranslator(src); }

IterativeOptimizationParameters::Kernel IterativeOptimizationParameters::kernelTranslator(const std::string &src) {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "CG") return IterativeOptimizationParameters::CG;
  /* default is cg */
  else return IterativeOptimizationParameters::CG;
}

IterativeOptimizationParameters::Verbosity IterativeOptimizationParameters::verbosityTranslator(const std::string &src)  {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "SILENT") return IterativeOptimizationParameters::SILENT;
  else if (s == "COMPLEXITY") return IterativeOptimizationParameters::COMPLEXITY;
  else if (s == "ERROR") return IterativeOptimizationParameters::ERROR;
  /* default is default */
  else return IterativeOptimizationParameters::SILENT;
}

std::string IterativeOptimizationParameters::kernelTranslator(IterativeOptimizationParameters::Kernel k)  {
  if ( k == CG ) return "CG";
  else return "UNKNOWN";
}

std::string IterativeOptimizationParameters::verbosityTranslator(IterativeOptimizationParameters::Verbosity verbosity)  {
  if (verbosity == SILENT) return "SILENT";
  else if (verbosity == COMPLEXITY) return "COMPLEXITY";
  else if (verbosity == ERROR) return "ERROR";
  else return "UNKNOWN";
}


}



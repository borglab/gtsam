/**
 * @file   NonlinearOptimizerParams.cpp
 * @brief  Parameters for nonlinear optimization
 * @date   Jul 24, 2012
 * @author Yong-Dian Jian
 * @author Richard Roberts
 * @author Frank Dellaert
 * @author Andrew Melim
 */

#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

namespace gtsam {

/* ************************************************************************* */
NonlinearOptimizerParams::Verbosity NonlinearOptimizerParams::verbosityTranslator(
    const std::string &src) {
  std::string s = src;
  // Convert to upper case
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
  if (s == "SILENT")
    return NonlinearOptimizerParams::SILENT;
  if (s == "ERROR")
    return NonlinearOptimizerParams::ERROR;
  if (s == "VALUES")
    return NonlinearOptimizerParams::VALUES;
  if (s == "DELTA")
    return NonlinearOptimizerParams::DELTA;
  if (s == "LINEAR")
    return NonlinearOptimizerParams::LINEAR;
  if (s == "TERMINATION")
    return NonlinearOptimizerParams::TERMINATION;

  /* default is silent */
  return NonlinearOptimizerParams::SILENT;
}

/* ************************************************************************* */
std::string NonlinearOptimizerParams::verbosityTranslator(
    Verbosity value) {
  std::string s;
  switch (value) {
  case NonlinearOptimizerParams::SILENT:
    s = "SILENT";
    break;
  case NonlinearOptimizerParams::TERMINATION:
    s = "TERMINATION";
    break;
  case NonlinearOptimizerParams::ERROR:
    s = "ERROR";
    break;
  case NonlinearOptimizerParams::VALUES:
    s = "VALUES";
    break;
  case NonlinearOptimizerParams::DELTA:
    s = "DELTA";
    break;
  case NonlinearOptimizerParams::LINEAR:
    s = "LINEAR";
    break;
  default:
    s = "UNDEFINED";
    break;
  }
  return s;
}

/* ************************************************************************* */
void NonlinearOptimizerParams::setIterativeParams(
    const std::shared_ptr<IterativeOptimizationParameters> params) {
  iterativeParams = params;
}

/* ************************************************************************* */
void NonlinearOptimizerParams::print(const std::string& str) const {

  //NonlinearOptimizerParams::print(str);
  std::cout << str << "\n";
  std::cout << "relative decrease threshold: " << relativeErrorTol << "\n";
  std::cout << "absolute decrease threshold: " << absoluteErrorTol << "\n";
  std::cout << "      total error threshold: " << errorTol << "\n";
  std::cout << "         maximum iterations: " << maxIterations << "\n";
  std::cout << "                  verbosity: " << verbosityTranslator(verbosity)
      << "\n";
  std::cout.flush();

  switch (linearSolverType) {
  case MULTIFRONTAL_SOLVER:
    std::cout << "         linear solver type: MULTIFRONTAL SOLVER\n";
    break;
  case MULTIFRONTAL_CHOLESKY:
    std::cout << "         linear solver type: MULTIFRONTAL CHOLESKY\n";
    break;
  case MULTIFRONTAL_QR:
    std::cout << "         linear solver type: MULTIFRONTAL QR\n";
    break;
  case SEQUENTIAL_CHOLESKY:
    std::cout << "         linear solver type: SEQUENTIAL CHOLESKY\n";
    break;
  case SEQUENTIAL_QR:
    std::cout << "         linear solver type: SEQUENTIAL QR\n";
    break;
  case CHOLMOD:
    std::cout << "         linear solver type: CHOLMOD\n";
    break;
  case Iterative:
    std::cout << "         linear solver type: ITERATIVE\n";
    break;
  default:
    std::cout << "         linear solver type: (invalid)\n";
    break;
  }

  if (linearSolverType == MULTIFRONTAL_SOLVER) {
    const auto& p = multifrontalParams;
    std::cout << "  multifrontal.leafMergeDimCap: " << p.leafMergeDimCap << "\n";
    std::cout << "  multifrontal.mergeDimCap: " << p.mergeDimCap << "\n";
    const char* qrMode = "off";
    switch (p.qrMode) {
    case MultifrontalParameters::QRMode::Off:
      qrMode = "off";
      break;
    case MultifrontalParameters::QRMode::Allow:
      qrMode = "allow";
      break;
    case MultifrontalParameters::QRMode::Force:
      qrMode = "force";
      break;
    }
    std::cout << "  multifrontal.qrMode: " << qrMode << "\n";
    std::cout << "  multifrontal.qrAspectRatio: " << p.qrAspectRatio << "\n";
    std::cout << "  multifrontal.eliminationParallelThreshold: "
              << p.eliminationParallelThreshold << "\n";
    std::cout << "  multifrontal.solutionParallelThreshold: "
              << p.solutionParallelThreshold << "\n";
    std::cout << "  multifrontal.numThreads: " << p.numThreads << "\n";
  }

  switch (orderingType){
  case Ordering::COLAMD:
    std::cout << "                   ordering: COLAMD\n";
    break;
  case Ordering::METIS:
    std::cout << "                   ordering: METIS\n";
    break;
  default:
    std::cout << "                   ordering: custom\n";
    break;
  }

  std::cout.flush();
}

/* ************************************************************************* */
bool NonlinearOptimizerParams::equals(const NonlinearOptimizerParams& other,
                                      double tol) const {
  // Check for equality of shared ptrs
  bool iterative_params_equal = iterativeParams == other.iterativeParams;
  // Check equality of components
  if (iterativeParams && other.iterativeParams) {
    iterative_params_equal = iterativeParams->equals(*other.iterativeParams);
  } else {
    // Check if either is null. If both are null, then true
    iterative_params_equal = !iterativeParams && !other.iterativeParams;
  }

  auto multifrontalEqual = [&]() {
    const auto& a = multifrontalParams;
    const auto& b = other.multifrontalParams;
    return a.leafMergeDimCap == b.leafMergeDimCap &&
           a.mergeDimCap == b.mergeDimCap &&
           a.qrMode == b.qrMode && a.qrAspectRatio == b.qrAspectRatio &&
           a.reportStream == b.reportStream &&
           a.eliminationParallelThreshold == b.eliminationParallelThreshold &&
           a.solutionParallelThreshold == b.solutionParallelThreshold &&
           a.numThreads == b.numThreads;
  };

  return maxIterations == other.getMaxIterations() &&
         std::abs(relativeErrorTol - other.getRelativeErrorTol()) <= tol &&
         std::abs(absoluteErrorTol - other.getAbsoluteErrorTol()) <= tol &&
         std::abs(errorTol - other.getErrorTol()) <= tol &&
         verbosityTranslator(verbosity) == other.getVerbosity() &&
         orderingType == other.orderingType && ordering == other.ordering &&
         linearSolverType == other.linearSolverType && iterative_params_equal &&
         multifrontalEqual();
}

/* ************************************************************************* */
std::string NonlinearOptimizerParams::linearSolverTranslator(
    LinearSolverType linearSolverType) const {
  switch (linearSolverType) {
  case MULTIFRONTAL_SOLVER:
    return "MULTIFRONTAL_SOLVER";
  case MULTIFRONTAL_CHOLESKY:
    return "MULTIFRONTAL_CHOLESKY";
  case MULTIFRONTAL_QR:
    return "MULTIFRONTAL_QR";
  case SEQUENTIAL_CHOLESKY:
    return "SEQUENTIAL_CHOLESKY";
  case SEQUENTIAL_QR:
    return "SEQUENTIAL_QR";
  case Iterative:
    return "ITERATIVE";
  case CHOLMOD:
    return "CHOLMOD";
  default:
    throw std::invalid_argument(
        "Unknown linear solver type in SuccessiveLinearizationOptimizer");
  }
}

/* ************************************************************************* */
NonlinearOptimizerParams::LinearSolverType NonlinearOptimizerParams::linearSolverTranslator(
    const std::string& linearSolverType) const {
  if (linearSolverType == "MULTIFRONTAL_SOLVER")
    return MULTIFRONTAL_SOLVER;
  if (linearSolverType == "MULTIFRONTAL_CHOLESKY")
    return MULTIFRONTAL_CHOLESKY;
  if (linearSolverType == "MULTIFRONTAL_QR")
    return MULTIFRONTAL_QR;
  if (linearSolverType == "SEQUENTIAL_CHOLESKY")
    return SEQUENTIAL_CHOLESKY;
  if (linearSolverType == "SEQUENTIAL_QR")
    return SEQUENTIAL_QR;
  if (linearSolverType == "ITERATIVE")
    return Iterative;
  if (linearSolverType == "CHOLMOD")
    return CHOLMOD;
  throw std::invalid_argument(
      "Unknown linear solver type in SuccessiveLinearizationOptimizer");
}

/* ************************************************************************* */
std::string NonlinearOptimizerParams::orderingTypeTranslator(
    Ordering::OrderingType type) const {
  switch (type) {
  case Ordering::METIS:
    return "METIS";
  case Ordering::COLAMD:
    return "COLAMD";
  default:
    if (ordering)
      return "CUSTOM";
    else
      throw std::invalid_argument(
          "Invalid ordering type: You must provide an ordering for a custom ordering type. See setOrdering");
  }
}

/* ************************************************************************* */
Ordering::OrderingType NonlinearOptimizerParams::orderingTypeTranslator(
    const std::string& type) const {
  if (type == "METIS")
    return Ordering::METIS;
  if (type == "COLAMD")
    return Ordering::COLAMD;
  throw std::invalid_argument(
      "Invalid ordering type: You must provide an ordering for a custom ordering type. See setOrdering");
}


} // namespace

/**
 * @file     LPState.h
 * @brief    This struct holds the state of QPSolver at each iteration    
 * @author   Ivan Dario Jimenez
 * @date     1/24/16
 */



namespace gtsam {

struct LPState {
  VectorValues values;
  VectorValues duals;
  InequalityFactorGraph workingSet;
  bool converged;
  size_t iterations;

  /// default constructor
  LPState() :
    values(), duals(), workingSet(), converged(false), iterations(0) {
  }

  /// constructor with initial values
  LPState(const VectorValues& initialValues, const VectorValues& initialDuals,
          const InequalityFactorGraph& initialWorkingSet, bool _converged,
          size_t _iterations) :
    values(initialValues), duals(initialDuals), workingSet(initialWorkingSet), converged(
    _converged), iterations(_iterations) {
  }
};

}
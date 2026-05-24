#include <gtsam/certifiable/LiftedSDPProblem.h>

#include <stdexcept>

namespace gtsam {

#ifdef GTSAM_USE_MOSEK
struct LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::Impl {
  KeyVector orderedKeys;

  void collectOrderedKeys(const QcqpProblem& problem) {
    const KeySet costKeys = problem.costs().keys();
    const KeySet eqKeys = problem.eConstraints().keys();
    const KeySet ineqKeys = problem.iConstraints().keys();

    KeySet constraintKeys = eqKeys;
    constraintKeys.merge(ineqKeys);

    // TODO: Does this always hold? 
    if (costKeys != constraintKeys) {
      throw std::runtime_error(
          "MonolithicSDP: QCQP constraint keys do not match objective cost keys.");
    }

    orderedKeys.assign(costKeys.begin(), costKeys.end());
  }
};

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::LiftedSDPProblem(
    const QcqpProblem& problem)
    : impl_(std::make_unique<Impl>()) {
  impl_->collectOrderedKeys(problem);
}

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::~LiftedSDPProblem() = default;
#endif

}  // namespace gtsam

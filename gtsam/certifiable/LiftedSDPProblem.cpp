#include <gtsam/certifiable/LiftedSDPProblem.h>

#include <stdexcept>

namespace gtsam {

#ifdef GTSAM_USE_MOSEK
namespace {

std::map<Key, DenseIndex> CollectQpCostKeyDims(const QcqpProblem& problem,
                                               KeySet* costKeys) {
  std::map<Key, DenseIndex> keyDims;
  for (const auto& factor : problem.costs()) {
    if (!factor) {
      continue;
    }

    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::runtime_error(
          "MonolithicSDP: expected objective factors to be QpCost.");
    }

    const HessianFactor& H = cost->hessianFactor();
    for (auto it = H.begin(); it != H.end(); ++it) {
      const Key key = *it;
      if (costKeys) {
        costKeys->insert(key);
      }
      const DenseIndex dim = H.getDim(it);
      const auto [entry, inserted] = keyDims.emplace(key, dim);
      if (!inserted && entry->second != dim) {
        throw std::runtime_error(
            "MonolithicSDP: inconsistent QpCost dimension for key.");
      }
    }
  }
  return keyDims;
}

}  // namespace

struct LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::Impl {
  KeyVector orderedKeys;
  std::map<Key, DenseIndex> orderedKeyDims;

  void collectOrderedKeysAndDims(const QcqpProblem& problem) {
    KeySet costKeys;
    orderedKeyDims = CollectQpCostKeyDims(problem, &costKeys);

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
  impl_->collectOrderedKeysAndDims(problem);
}

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::~LiftedSDPProblem() = default;

const KeyVector&
LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::orderedKeys() const {
  return impl_->orderedKeys;
}

const std::map<Key, DenseIndex>&
LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::orderedKeyDims() const {
  return impl_->orderedKeyDims;
}
#endif

}  // namespace gtsam

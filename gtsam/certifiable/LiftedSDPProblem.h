#pragma once

#include <gtsam/config.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/inference/Key.h>

#include <map>
#include <memory>

namespace gtsam {

struct MonolithicSDP {};

struct MosekSDPSolver {};

template <typename SDPFormulation, typename SDPSolver>
class LiftedSDPProblem;

#ifdef GTSAM_USE_MOSEK
template <>
class LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> {
 public:

  explicit LiftedSDPProblem(const QcqpProblem& problem);

  ~LiftedSDPProblem();

  const KeyVector& orderedKeys() const;

  const std::map<Key, DenseIndex>& orderedKeyDims() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
#endif

}  // namespace gtsam

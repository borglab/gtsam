/* #pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/symbolic/SymbolicBayesTree.h>
#include <map>
#include <memory>
#include <string>
#include <vector> */
#pragma once

#include <gtsam/config.h>
#include <gtsam/constrained/QcqpProblem.h>

#include <memory>

namespace gtsam {

/** Formulation tag selecting a single dense lifted SDP variable. */
struct MonolithicSDP {};

/** Solver tag selecting the optional MOSEK-backed SDP implementation. */
struct MosekSDPSolver {};

/** Forward declaration required before formulation/solver specializations. */
template <typename SDPFormulation, typename SDPSolver>
class LiftedSDPProblem;

#ifdef GTSAM_USE_MOSEK
template <>
class LiftedSDPProblem<MonolithicSDP, MosekSDPSolver> {
 public:

  explicit LiftedSDPProblem(const QcqpProblem& problem);

  ~LiftedSDPProblem();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
#endif

}  // namespace gtsam

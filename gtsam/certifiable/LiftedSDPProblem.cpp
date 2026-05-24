#include <gtsam/certifiable/LiftedSDPProblem.h>

namespace gtsam {

#ifdef GTSAM_USE_MOSEK
struct LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::Impl {};

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::LiftedSDPProblem(
    const QcqpProblem& problem)
    : impl_(std::make_unique<Impl>()) {
  (void)problem;
}

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::~LiftedSDPProblem() = default;
#endif

}  // namespace gtsam

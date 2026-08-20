#pragma once

#include <gtsam/linear/cuda/LinearSolver.h>

namespace gtsam::cuda {

/// LM action after a device PCG solve that did not necessarily converge.
enum class PcgLmStepDisposition {
  /// Evaluate a converged or finite iteration-limited delta as an LM step.
  EvaluateInexactStep,
  /// Discard a numerically broken recurrence and retry with more damping.
  RejectAndRetry,
};

/**
 * Classifies a PCG result for LM.
 *
 * Reaching the iteration cap still produces a finite inexact step whose
 * quality is checked by LM's predicted/actual reduction test. A numerical
 * breakdown invalidates that step before model or nonlinear evaluation.
 */
inline PcgLmStepDisposition classifyPcgLmStep(
    const LinearSolveStats& stats) {
  return stats.lastPcgBreakdown ? PcgLmStepDisposition::RejectAndRetry
                                : PcgLmStepDisposition::EvaluateInexactStep;
}

}  // namespace gtsam::cuda

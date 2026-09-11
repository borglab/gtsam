/* Copyright 2026, Georgia Tech Research Corporation. See LICENSE. */
#pragma once

#include <gtsam/constrained/QcqpProblem.h>

#include <map>
#include <vector>

namespace gtsam::internal {

/**
 * Post-process completed clique layouts, leaving the QCQP unchanged.
 * Each returned B represents the original clique moment matrix as B Z B'.
 * Only proven duplicate homogeneous rows and individually fixed coordinates
 * are eliminated; other constraints remain for the solver to enforce.
 */
GTSAM_EXPORT std::map<KeyVector, Matrix> EliminateKnownNullDirections(
    const QcqpProblem& problem, const std::map<Key, DenseIndex>& dimensions,
    const std::vector<KeyVector>& cliques);

}  // namespace gtsam::internal

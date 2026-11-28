// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file AdamUpdate.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#pragma once

#include <gtsam/base/Matrix.h>

#include <cmath>
#include <initializer_list>

namespace gtsam::internal {

/// One matrix or bias vector and its accumulated minibatch gradient.
struct AdamParameterBlock {
  Eigen::Ref<Matrix> parameters;
  Eigen::Ref<const Matrix> gradient;
};

/// Apply one bias-corrected Adam step to all blocks of a fixed-size network.
/// The caller supplies the minibatch normalization, optionally including norm
/// clipping. A nonzero decay applies decoupled weight decay (AdamW).
inline void adamUpdate(Vector& firstMoment, Vector& secondMoment,
                       size_t& updates,
                       std::initializer_list<AdamParameterBlock> blocks,
                       double learningRate, double denominator,
                       double decay = 0.) {
  if (!firstMoment.size()) {
    Eigen::Index size = 0;
    for (const auto& block : blocks) size += block.parameters.size();
    firstMoment = Vector::Zero(size);
    secondMoment = Vector::Zero(size);
  }
  ++updates;
  const double correction = std::sqrt(1. - std::pow(0.999, updates));
  const double rate = learningRate * correction / (1. - std::pow(0.9, updates));
  Eigen::Index offset = 0;
  for (auto block : blocks) {
    const auto rows = block.parameters.rows(), cols = block.parameters.cols();
    Eigen::Map<Matrix> first(firstMoment.data() + offset, rows, cols);
    Eigen::Map<Matrix> second(secondMoment.data() + offset, rows, cols);
    const auto gradient = block.gradient.array() / denominator;
    first.array() += 0.1 * (gradient - first.array());
    second.array() += 0.001 * (gradient.square() - second.array());
    if (decay) block.parameters *= 1. - learningRate * decay;
    block.parameters.array() -=
        rate * first.array() / (second.array().sqrt() + 1.e-8 * correction);
    offset += block.parameters.size();
  }
}

}  // namespace gtsam::internal

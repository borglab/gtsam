/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    JointMarginal.cpp
 * @brief   Block access to joint Gaussian covariance or information matrices
 * @author  Codex
 */

#include <gtsam/linear/JointMarginal.h>

#include <iostream>

namespace gtsam {

namespace {

/* ************************************************************************* */
std::vector<size_t> dimsFromScatter(const Scatter& scatter) {
  std::vector<size_t> dims;
  dims.reserve(scatter.size());
  for (const auto& slot : scatter) {
    dims.push_back(slot.dimension);
  }
  return dims;
}

/* ************************************************************************* */
KeyVector keysFromScatter(const Scatter& scatter) {
  KeyVector keys;
  keys.reserve(scatter.size());
  for (const auto& slot : scatter) {
    keys.push_back(slot.key);
  }
  return keys;
}

/* ************************************************************************* */
FastMap<Key, size_t> indicesFromScatter(const Scatter& scatter) {
  FastMap<Key, size_t> indices;
  size_t index = 0;
  for (const auto& slot : scatter) {
    indices[slot.key] = index++;
  }
  return indices;
}

}  // namespace

/* ************************************************************************* */
JointMarginal::JointMarginal(const Matrix& fullMatrix, const Scatter& scatter)
    : blockMatrix_(dimsFromScatter(scatter), fullMatrix),
      keys_(keysFromScatter(scatter)),
      indices_(indicesFromScatter(scatter)) {}

/* ************************************************************************* */
void JointMarginal::print(const std::string& s,
                          const KeyFormatter& formatter) const {
  std::cout << s << "Joint marginal on keys ";
  bool first = true;
  for (const auto& key : keys_) {
    if (!first)
      std::cout << ", ";
    else
      first = false;
    std::cout << formatter(key);
  }
  std::cout << ".  Use 'at' or 'operator()' to query matrix blocks."
            << std::endl;
}

}  // namespace gtsam

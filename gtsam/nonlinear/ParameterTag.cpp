/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */

#include <gtsam/nonlinear/ParameterTag.h>

#include <iostream>
#include <utility>

namespace gtsam {

ParameterTag::ParameterTag(std::string name)
    : identity_(std::make_shared<Identity>(Identity{std::move(name)})) {}

void ParameterTag::print(const std::string& prefix) const {
  std::cout << prefix << name() << '\n';
}

}  // namespace gtsam

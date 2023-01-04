/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridNonlinearFactor.cpp
 *  @date May 28, 2022
 *  @author Varun Agrawal
 */

#include <gtsam/hybrid/HybridNonlinearFactor.h>

#include <boost/make_shared.hpp>

namespace gtsam {

/* ************************************************************************* */
HybridNonlinearFactor::HybridNonlinearFactor(
    const NonlinearFactor::shared_ptr &other)
    : Base(other->keys()), inner_(other) {}

/* ************************************************************************* */
bool HybridNonlinearFactor::equals(const HybridFactor &lf, double tol) const {
  return Base::equals(lf, tol);
}

/* ************************************************************************* */
void HybridNonlinearFactor::print(const std::string &s,
                                  const KeyFormatter &formatter) const {
  HybridFactor::print(s, formatter);
  inner_->print("\n", formatter);
};

}  // namespace gtsam

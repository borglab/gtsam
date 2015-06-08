/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * LinearEqualityFactorGraph.h
 * @brief: Factor graph of all LinearEquality factors
 * @date: Dec 8, 2014
 * @author: thduynguyen
 */

#pragma once

#include <gtsam/inference/FactorGraph.h>
#include <gtsam_unstable/linear/LinearEquality.h>

namespace gtsam {

class LinearEqualityFactorGraph : public FactorGraph<LinearEquality> {
public:
  typedef boost::shared_ptr<LinearEqualityFactorGraph> shared_ptr;
};

/// traits
template<> struct traits<LinearEqualityFactorGraph> : public Testable<
    LinearEqualityFactorGraph> {
};

} // \ namespace gtsam


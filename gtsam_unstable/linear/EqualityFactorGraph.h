/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * EqualityFactorGraph.h
 * @brief: Factor graph of all LinearEquality factors
 * @date: Dec 8, 2014
 * @author: Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/inference/FactorGraph.h>
#include <gtsam_unstable/linear/LinearEquality.h>

namespace gtsam {

class EqualityFactorGraph : public FactorGraph<LinearEquality> {
public:
  typedef boost::shared_ptr<EqualityFactorGraph> shared_ptr;
};

/// traits
template<> struct traits<EqualityFactorGraph> : public Testable<
    EqualityFactorGraph> {
};

} // \ namespace gtsam


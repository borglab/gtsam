/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    inference.h
 * @brief   Contains *generic* inference algorithms that convert between templated
 * graphical models, i.e., factor graphs, Bayes nets, and Bayes trees
 * @author  Frank Dellaert, Richard Roberts
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/Permutation.h>

#include <vector>
#include <deque>

namespace gtsam {

	class Inference {
	private:
	  /* Static members only, private constructor */
	  Inference() {}

	  // Internal version that actually calls colamd after the constraint set is created in the right format
    static Permutation::shared_ptr PermutationCOLAMD_(const VariableIndex& variableIndex, std::vector<int>& cmember);

	public:

    /**
     * Compute a permutation (variable ordering) using colamd
     */
    static Permutation::shared_ptr PermutationCOLAMD(const VariableIndex& variableIndex);

    /**
     * Compute a permutation (variable ordering) using constrained colamd
     */
    template<typename CONSTRAINED>
    static Permutation::shared_ptr PermutationCOLAMD(const VariableIndex& variableIndex, const CONSTRAINED& constrainLast);

	};

} /// namespace gtsam

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

#include <gtsam/base/FastVector.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/Permutation.h>

#include <boost/foreach.hpp>

#include <deque>

namespace gtsam {

	class Inference {
	private:
	  /* Static members only, private constructor */
	  Inference() {}

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

	  /**
	   * Compute a CCOLAMD permutation using the constraint groups in cmember.
	   */
    static Permutation::shared_ptr PermutationCOLAMD_(const VariableIndex& variableIndex, FastVector<int>& cmember);

	};

	/* ************************************************************************* */
	template<typename CONSTRAINED>
	Permutation::shared_ptr Inference::PermutationCOLAMD(const VariableIndex& variableIndex, const CONSTRAINED& constrainLast) {

	  FastVector<int> cmember(variableIndex.size(), 0);

	  // If at least some variables are not constrained to be last, constrain the
	  // ones that should be constrained.
	  if(constrainLast.size() < variableIndex.size()) {
	    BOOST_FOREACH(Index var, constrainLast) {
	      assert(var < variableIndex.size());
	      cmember[var] = 1;
	    }
	  }

	  return PermutationCOLAMD_(variableIndex, cmember);
	}

	/* ************************************************************************* */
	inline Permutation::shared_ptr Inference::PermutationCOLAMD(const VariableIndex& variableIndex) {
	  FastVector<int> cmember(variableIndex.size(), 0);
	  return PermutationCOLAMD_(variableIndex, cmember);
	}

} // namespace gtsam

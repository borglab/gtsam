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

	public:

    /**
     * Compute a permutation (variable ordering) using colamd
     */
    template<class VARIABLEINDEXTYPE>
    static boost::shared_ptr<Permutation> PermutationCOLAMD(const VARIABLEINDEXTYPE& variableIndex) { return PermutationCOLAMD(variableIndex, std::vector<Index>()); }
    template<class VARIABLEINDEXTYPE, typename CONSTRAINTCONTAINER>
    static boost::shared_ptr<Permutation> PermutationCOLAMD(const VARIABLEINDEXTYPE& variableIndex, const CONSTRAINTCONTAINER& constrainLast);

	};

	// ELIMINATE: FACTOR GRAPH -> BAYES NET

//	/**
//   * Eliminate a single node yielding a Conditional
//   * Eliminates the factors from the factor graph through findAndRemoveFactors
//   * and adds a new factor on the separator to the factor graph
//   */
//	template<class Factor, class Conditional>
//	boost::shared_ptr<Conditional>
//	eliminateOne(FactorGraph<Factor>& factorGraph, Index key);
//
//	/**
//	 * eliminate factor graph using the given (not necessarily complete)
//	 * ordering, yielding a chordal Bayes net and (partially eliminated) FG
//	 */
//	template<class Factor, class Conditional>
//	BayesNet<Conditional> eliminate(FactorGraph<Factor>& factorGraph, const Ordering& ordering);

	// FACTOR/MARGINALIZE: BAYES NET -> FACTOR GRAPH

//	/**
//	 * Factor P(X) as P(not keys|keys) P(keys)
//	 * @return P(not keys|keys) as an incomplete BayesNet, and P(keys) as a factor graph
//	 */
//	template<class Factor, class Conditional>
//	std::pair< BayesNet<Conditional>, FactorGraph<Factor> >
//	factor(const BayesNet<Conditional>& bn, const Ordering& keys);
//
//	/**
//	 * integrate out all except ordering, might be inefficient as the ordering
//	 * will simply be the current ordering with the keys put in the back
//	 */
//	template<class Factor, class Conditional>
//	FactorGraph<Factor> marginalize(const BayesNet<Conditional>& bn, const Ordering& keys);

	/**
	 * Hacked-together function to compute a Gaussian marginal for the given variable.
	 * todo: This is inefficient!
	 */
	//std::pair<Vector,Matrix> marginalGaussian(const GaussianFactorGraph& fg, const Symbol& key);

} /// namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Potentials.h
 * @date March 24, 2011
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/AlgebraicDecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/base/types.h>
#include <gtsam/inference/Permutation.h>

#include <boost/shared_ptr.hpp>
#include <set>

namespace gtsam {

	/**
	 * A base class for both DiscreteFactor and DiscreteConditional
	 */
	class Potentials: public AlgebraicDecisionTree<Index> {

	public:

		typedef AlgebraicDecisionTree<Index> ADT;

	protected:

		/// Cardinality for each key, used in combine
		std::map<Index,size_t> cardinalities_;

		/** Constructor from ColumnIndex, and ADT */
		Potentials(const ADT& potentials) :
				ADT(potentials) {
		}

		// Safe division for probabilities
		static double safe_div(const double& a, const double& b);

	public:

		/** Default constructor for I/O */
		Potentials();

		/** Constructor from Indices and ADT */
		Potentials(const DiscreteKeys& keys, const ADT& decisionTree);

		/** Constructor from Indices and (string or doubles) */
		template<class SOURCE>
		Potentials(const DiscreteKeys& keys, SOURCE table) :
				ADT(keys, table), cardinalities_(keys.cardinalities()) {
		}

		// Testable
		bool equals(const Potentials& other, double tol = 1e-9) const;
		void print(const std::string& s = "Potentials: ",
				const IndexFormatter& formatter = DefaultIndexFormatter) const;

		size_t cardinality(Index j) const { return cardinalities_.at(j);}

		/**
		 * @brief Permutes the keys in Potentials
		 *
		 * This permutes the Indices and performs necessary re-ordering of ADD.
		 * This is virtual so that derived types e.g. DecisionTreeFactor can
		 * re-implement it.
		 */
		virtual void permuteWithInverse(const Permutation& inversePermutation);

	}; // Potentials

} // namespace gtsam

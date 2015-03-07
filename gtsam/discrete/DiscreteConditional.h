/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteConditional.h
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/inference/IndexConditional.h>
#include <boost/shared_ptr.hpp>

namespace gtsam {

	/**
	 * Discrete Conditional Density
	 * Derives from DecisionTreeFactor
	 */
	class DiscreteConditional: public IndexConditional, public Potentials {

	public:
		// typedefs needed to play nice with gtsam
		typedef DiscreteFactor FactorType;
		typedef boost::shared_ptr<DiscreteConditional> shared_ptr;
		typedef IndexConditional Base;

		/** A map from keys to values */
		typedef Assignment<Index> Values;
		typedef boost::shared_ptr<Values> sharedValues;

		/// @name Standard Constructors
		/// @{

		/** default constructor needed for serialization */
		DiscreteConditional() {
		}

		/** constructor from factor */
		DiscreteConditional(size_t nFrontals, const DecisionTreeFactor& f);

		/** Construct from signature */
		DiscreteConditional(const Signature& signature);

		/** construct P(X|Y)=P(X,Y)/P(Y) from P(X,Y) and P(Y) */
		DiscreteConditional(const DecisionTreeFactor& joint,
				const DecisionTreeFactor& marginal);

		/// @}
		/// @name Testable
		/// @{

		/** GTSAM-style print */
		void print(const std::string& s = "Discrete Conditional: ",
				const IndexFormatter& formatter
				=DefaultIndexFormatter) const {
			std::cout << s << std::endl;
			IndexConditional::print(s, formatter);
			Potentials::print(s);
		}

		/** GTSAM-style equals */
		bool equals(const DiscreteConditional& other, double tol = 1e-9) const {
			return IndexConditional::equals(other, tol)
					&& Potentials::equals(other, tol);
		}

		/// @}
		/// @name Standard Interface
		/// @{

		/** Convert to a factor */
		DecisionTreeFactor::shared_ptr toFactor() const {
			return DecisionTreeFactor::shared_ptr(new DecisionTreeFactor(*this));
		}

		/** Restrict to given parent values, returns AlgebraicDecisionDiagram */
		ADT choose(const Assignment<Index>& parentsValues) const;

		/**
		 * solve a conditional
		 * @param parentsValues Known values of the parents
		 * @return MPE value of the child (1 frontal variable).
		 */
		size_t solve(const Values& parentsValues) const;

		/**
		 * sample
		 * @param parentsValues Known values of the parents
		 * @return sample from conditional
		 */
		size_t sample(const Values& parentsValues) const;

		/// @}
		/// @name Advanced Interface
		/// @{

		/// solve a conditional, in place
		void solveInPlace(Values& parentsValues) const;

		/// sample in place, stores result in partial solution
		void sampleInPlace(Values& parentsValues) const;

		/**
		 * Permutes both IndexConditional and Potentials.
		 */
		void permuteWithInverse(const Permutation& inversePermutation);

		/// @}

	};
// DiscreteConditional

}// gtsam


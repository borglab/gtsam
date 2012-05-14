/*
 * @file		TypedDiscreteFactor.h
 * @brief		
 * @author	Duy-Nguyen Ta
 * @date	Mar 5, 2011
 */

#pragma once

#include <map>
#include <gtsam/inference/Factor.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/AlgebraicDecisionDiagram.h>

namespace gtsam {

	/**
	 * A factor on discrete variables with string keys
	 */
	class TypedDiscreteFactor: public Factor<Index> {

		typedef AlgebraicDecisionDiagram<Index> ADD;

		/** potentials of the factor */
		ADD potentials_;

	public:

		/** A map from keys to values */
		typedef ADD::Assignment Values;

		/** Constructor from keys and string table */
		TypedDiscreteFactor(const Indices& keys, const std::string& table);

		/** Constructor from keys and doubles */
		TypedDiscreteFactor(const Indices& keys,
				const std::vector<double>& table);

		/** Evaluate */
		double operator()(const Values& values) const;

		// Testable
		bool equals(const TypedDiscreteFactor& other, double tol = 1e-9) const;
		void print(const std::string& s = "DiscreteFactor: ") const;

		DiscreteFactor::shared_ptr toDiscreteFactor(const KeyOrdering& ordering) const;

#ifdef OLD
		/** map each variable name to its column index in the potential table */
		typedef std::map<std::string, size_t> Index2IndexMap;
		Index2IndexMap columnIndex_;

		/** Initialize keys, column index, and return cardinalities */
		std::vector<size_t> init(const Indices& keys);

	public:

		/** Default constructor */
		TypedDiscreteFactor() {}

		/** Evaluate potential of a given assignment of values */
		double potential(const TypedValues& values) const;

#endif

	}; // TypedDiscreteFactor

} // namespace

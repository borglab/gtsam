/*
 * @file		TypedDiscreteFactorGraph.h
 * @brief		Factor graph with typed factors (with Index keys)
 * @author	Duy-Nguyen Ta
 * @author	Frank Dellaert
 * @date	Mar 1, 2011
 */

#pragma once

#include <gtsam_unstable/discrete/TypedDiscreteFactor.h>
#include <gtsam/inference/FactorGraph.h>
#include <vector>
#include <set>

namespace gtsam {

	/**
	 * Typed discrete factor graph, where keys are strings
	 */
	class TypedDiscreteFactorGraph: public FactorGraph<TypedDiscreteFactor> {

	public:

		/**
		 * Default constructor
		 */
		TypedDiscreteFactorGraph();

		/**
		 * Constructor from file
		 * For now assumes in .uai format from UAI'08 Probablistic Inference Evaluation
		 * See http://graphmod.ics.uci.edu/uai08/FileFormat
		 */
		TypedDiscreteFactorGraph(const std::string& filename);

		// Add factors without shared pointer ugliness
		void add(const Indices& keys, const std::string& table);
		void add(const Indices& keys, const std::vector<double>& table);

		/** print */
		void print(const std::string s);

		/** Evaluate potential of a given assignment of values */
		double operator()(const TypedDiscreteFactor::Values& values) const;

	}; // TypedDiscreteFactorGraph


} // namespace

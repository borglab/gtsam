/**
 * @file    FactorGraph.h
 * @brief   Factor Graph Base Class
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "Testable.h"

namespace gtsam {

	class Ordering;
	class VectorConfig;
	class LinearFactor;
	class LinearFactorGraph;
	class Ordering;

	/**
	 * A factor graph is a bipartite graph with factor nodes connected to variable nodes.
	 * In this class, however, only factor nodes are kept around.
	 * 
	 * Templated on the type of factors and configuration.
	 */
	template<class Factor, class Config> class FactorGraph
		: public Testable<FactorGraph<Factor,Config> >
	{
	public:
		typedef typename boost::shared_ptr<Factor> shared_factor;
		typedef typename std::vector<shared_factor>::iterator iterator;
		typedef typename std::vector<shared_factor>::const_iterator const_iterator;

	protected:
		/** Collection of factors */
		std::vector<shared_factor> factors_;

		/** For each variable a list of factor indices connected to it  */
		typedef std::map<std::string, std::list<int> > Indices;
		Indices indices_;

	public:

		/** print out graph */
		void print(const std::string& s = "FactorGraph") const;

		/** Check equality */
		bool equals(const FactorGraph& fg, double tol = 1e-9) const;

		/** STL like, return the iterator pointing to the first factor */
		inline const_iterator begin() const {
			return factors_.begin();
		}

		/** STL like, return the iterator pointing to the last factor */
		inline const_iterator end() const {
			return factors_.end();
		}

		/** clear the factor graph */
		inline void clear() {
			factors_.clear();
		}

		/** Get a specific factor by index */
		inline shared_factor operator[](size_t i) const {
			return factors_[i];
		}

		/** return the numbers of the factors_ in the factor graph */
		inline size_t size() const {
			int size_=0;
			for (const_iterator factor = factors_.begin(); factor != factors_.end(); factor++)
			  if(*factor != NULL)
				  size_++;
			return size_;
		}

		/** Add a factor */
		void push_back(shared_factor factor);

		/** unnormalized error */
		double error(const Config& c) const {
			double total_error = 0.;
			/** iterate over all the factors_ to accumulate the log probabilities */
			for (const_iterator factor = factors_.begin(); factor != factors_.end(); factor++)
				total_error += (*factor)->error(c);

			return total_error;
		}

		/** Unnormalized probability. O(n) */
		double probPrime(const Config& c) const {
			return exp(-0.5 * error(c));
		}

		/**
		 * Compute colamd ordering
		 */
		Ordering getOrdering() const;

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(factors_);
		}
	}; // FactorGraph
} // namespace gtsam


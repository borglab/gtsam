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

namespace gtsam {

	class Ordering;
	class FGConfig;
	class LinearFactor;
	class LinearFactorGraph;
	class Ordering;

	/**
	 * A factor graph is a bipartite graph with factor nodes connected to variable nodes.
	 * In this class, however, only factor nodes are kept around.
	 * 
	 * Templated on the type of factors and configuration.
	 */
	template<class Factor, class Config> class FactorGraph {
	public:
		typedef typename boost::shared_ptr<Factor> shared_factor;
		typedef typename std::vector<shared_factor>::iterator iterator;
		typedef typename std::vector<shared_factor>::const_iterator const_iterator;

	protected:
		/** Collection of factors */
		std::vector<shared_factor> factors_;

	public:

		/** STL like, return the iterator pointing to the first factor */
		const_iterator begin() const {
			return factors_.begin();
		}

		/** STL like, return the iterator pointing to the last factor */
		const_iterator end() const {
			return factors_.end();
		}

		/** clear the factor graph */
		void clear() {
			factors_.clear();
		}

		/** Get a specific factor by index */
		shared_factor operator[](size_t i) const {
			return factors_[i];
		}

		/** return the numbers of the factors_ in the factor graph */
		inline size_t size() const {
			return factors_.size();
		}

		/** Add a factor */
		void push_back(shared_factor ptr_f) {
			factors_.push_back(ptr_f);
		}

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

		/** print out graph */
		void print(const std::string& s = "FactorGraph") const {
			std::cout << s << std::endl;
			printf("size: %d\n", (int) size());
			for (const_iterator factor = factors_.begin(); factor != factors_.end(); factor++)
				(*factor)->print();
		}

		/** Check equality */
		bool equals(const FactorGraph& fg, double tol = 1e-9) const {
			/** check whether the two factor graphs have the same number of factors_ */
			if (factors_.size() != fg.size()) goto fail;

			/** check whether the factors_ are the same */
			for (size_t i = 0; i < factors_.size(); i++)
				// TODO: Doesn't this force order of factor insertion?
				if (!factors_[i]->equals(*fg.factors_[i], tol)) goto fail;
			return true;

			fail: print();
			fg.print();
			return false;
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


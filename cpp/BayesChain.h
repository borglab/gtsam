/**
 * @file    BayesChain
 * @brief   Bayes Chain, the result of eliminating a factor graph
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>

#include "Testable.h"

namespace gtsam {

	/**
	 * Bayes Chain, the result of eliminating a factor graph
	 * This is the base class for SymbolicBayesChain, DiscreteBayesChain, and GaussianBayesChain
	 * Corresponding to what is used for the "Conditional" template argument:
	 * a ConditionalProbabilityTable, a ConditionalGaussian, or a SymbolicConditional.
	 */
	template<class Conditional>
	class BayesChain: public Testable<BayesChain<Conditional> > {
	protected:

		/** nodes keys stored in topological sort order, i.e. from parents to children */
		std::list<std::string> keys_;

		/** nodes stored on key */
		typedef typename std::map<std::string, boost::shared_ptr<Conditional> > Nodes;
		Nodes nodes_;

	public:

		/** print */
		void print(const std::string& s = "") const;

		/** check equality */
		bool equals(const BayesChain& other, double tol = 1e-9) const;

		/** insert: use reverse topological sort (i.e. parents last) */
		void insert(const std::string& key, boost::shared_ptr<Conditional> node);

		/** delete */
		void erase(const std::string& key);

		/** size is the number of nodes */
		inline size_t size() const {return nodes_.size();}

		/** return keys in topological sort order (parents first), i.e., reverse elimination order */
		inline std::list<std::string> keys() const { return keys_;}

		inline boost::shared_ptr<Conditional> operator[](const std::string& key) const {
			const_iterator cg = nodes_.find(key); // get node
			assert( cg != nodes_.end() );
			return cg->second;
		}

		/** return begin and end of the nodes. FD: breaks encapsulation? */
		typedef typename Nodes::const_iterator const_iterator;
		const_iterator const begin() const {return nodes_.begin();}
		const_iterator const end()   const {return nodes_.end();}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & BOOST_SERIALIZATION_NVP(keys_);
			ar & BOOST_SERIALIZATION_NVP(nodes_);
		}
	};

} /// namespace gtsam

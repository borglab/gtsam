/**
 * @file   inference-inl.h
 * @brief  inference template definitions
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/inference/inference.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/Key.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	/* eliminate one node from the factor graph                           */
	/* ************************************************************************* */
	template<class Factor,class Conditional>
	boost::shared_ptr<Conditional> eliminateOne(FactorGraph<Factor>& graph, const Symbol& key) {

		// combine the factors of all nodes connected to the variable to be eliminated
		// if no factors are connected to key, returns an empty factor
		boost::shared_ptr<Factor> joint_factor = removeAndCombineFactors(graph,key);

		// eliminate that joint factor
		boost::shared_ptr<Factor> factor;
		boost::shared_ptr<Conditional> conditional;
		boost::tie(conditional, factor) = joint_factor->eliminate(key);

		// add new factor on separator back into the graph
		if (!factor->empty()) graph.push_back(factor);

		// return the conditional Gaussian
		return conditional;
	}

	/* ************************************************************************* */
	// This doubly templated function is generic. There is a GaussianFactorGraph
	// version that returns a more specific GaussianBayesNet.
	// Note, you will need to include this file to instantiate the function.
	/* ************************************************************************* */
	template<class Factor,class Conditional>
	BayesNet<Conditional> eliminate(FactorGraph<Factor>& factorGraph, const Ordering& ordering)
	{
		BayesNet<Conditional> bayesNet; // empty

		BOOST_FOREACH(Symbol key, ordering) {
			boost::shared_ptr<Conditional> cg = eliminateOne<Factor,Conditional>(factorGraph,key);
			bayesNet.push_back(cg);
		}

		return bayesNet;
	}

	/* ************************************************************************* */
	template<class Factor, class Conditional>
	pair< BayesNet<Conditional>, FactorGraph<Factor> >
	factor(const BayesNet<Conditional>& bn, const Ordering& keys) {
		// Convert to factor graph
		FactorGraph<Factor> factorGraph(bn);

		// Get the keys of all variables and remove all keys we want the marginal for
		Ordering ord = bn.ordering();
		BOOST_FOREACH(const Symbol& key, keys) ord.remove(key); // TODO: O(n*k), faster possible?

		// eliminate partially,
		BayesNet<Conditional> conditional = eliminate<Factor,Conditional>(factorGraph,ord);

		// at this moment, the factor graph only encodes P(keys)
		return make_pair(conditional,factorGraph);
		}

	/* ************************************************************************* */
	template<class Factor, class Conditional>
	FactorGraph<Factor> marginalize(const BayesNet<Conditional>& bn, const Ordering& keys) {

		// factor P(X,Y) as P(X|Y)P(Y), where Y corresponds to  keys
		pair< BayesNet<Conditional>, FactorGraph<Factor> > factors =
				gtsam::factor<Factor,Conditional>(bn,keys);

		// throw away conditional, return marginal P(Y)
		return factors.second;
		}

	/* ************************************************************************* */
//	pair<Vector,Matrix> marginalGaussian(const GaussianFactorGraph& fg, const Symbol& key) {
//
//		// todo: this does not use colamd!
//
//		list<Symbol> ord;
//		BOOST_FOREACH(const Symbol& k, fg.keys()) {
//			if(k != key)
//				ord.push_back(k);
//		}
//		Ordering ordering(ord);
//
//		// Now make another factor graph where we eliminate all the other variables
//		GaussianFactorGraph marginal(fg);
//		marginal.eliminate(ordering);
//
//		GaussianFactor::shared_ptr factor;
//		for(size_t i=0; i<marginal.size(); i++)
//			if(marginal[i] != NULL) {
//				factor = marginal[i];
//				break;
//			}
//
//		if(factor->keys().size() != 1 || factor->keys().front() != key)
//			throw runtime_error("Didn't get the right marginal!");
//
//		VectorConfig mean_cfg(marginal.optimize(Ordering(key)));
//		Matrix A(factor->get_A(key));
//
//		return make_pair(mean_cfg[key], inverse(prod(trans(A), A)));
//	}

	/* ************************************************************************* */

} // namespace gtsam

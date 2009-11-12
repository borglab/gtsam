/**
 * @file   inference-inl.h
 * @brief  inference template definitions
 * @author Frank Dellaert
 */

#include "inference.h"
#include "FactorGraph-inl.h"
#include "BayesNet-inl.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	/* eliminate one node from the factor graph                           */
	/* ************************************************************************* */
	template<class Factor,class Conditional>
	boost::shared_ptr<Conditional> eliminateOne(FactorGraph<Factor>& graph, const string& key) {

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

		BOOST_FOREACH(string key, ordering) {
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
		BOOST_FOREACH(string key, keys) ord.remove(key); // TODO: O(n*k), faster possible?

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

} // namespace gtsam

/**
 * @file   FactorGraph-inl.h
 * This is a template definition file, include it where needed (only!)
 * so that the appropriate code is generated and link errors avoided.
 * @brief  Factor Graph Base Class
 * @author Carlos Nieto
 * @author Frank Dellaert
 * @author Alireza Fathi
 * @author Michael Kaess
 */

#pragma once

#include <list>
#include <sstream>
#include <stdexcept>
#include <functional>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/format.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include "ccolamd.h"
#include "Ordering.h"
#include "FactorGraph.h"
#include "graph-inl.h"
#include "DSF.h"

#define INSTANTIATE_FACTOR_GRAPH(F) \
  template class FactorGraph<F>; \
  /*template boost::shared_ptr<F> removeAndCombineFactors(FactorGraph<F>&, const std::string&);*/ \
  template FactorGraph<F> combine(const FactorGraph<F>&, const FactorGraph<F>&);

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class Factor>
	void FactorGraph<Factor>::associateFactor(size_t index,
			const sharedFactor& factor) {
		// rtodo: Can optimize factor->keys to return a const reference
		const list<Symbol> keys = factor->keys(); // get keys for factor

		// for each key push i onto list
		BOOST_FOREACH(const Symbol& key, keys)
						indices_[key].push_back(index);
	}

	/* ************************************************************************* */
	template<class Factor>
	template<class Conditional>
	FactorGraph<Factor>::FactorGraph(const BayesNet<Conditional>& bayesNet) {
		typename BayesNet<Conditional>::const_iterator it = bayesNet.begin();
		for (; it != bayesNet.end(); it++) {
			sharedFactor factor(new Factor(*it));
			push_back(factor);
		}
	}

	/* ************************************************************************* */
	template<class Factor>
	void FactorGraph<Factor>::push_back(sharedFactor factor) {
		factors_.push_back(factor); // add the actual factor
		if (factor == NULL) return;

		size_t i = factors_.size() - 1; // index of factor
		associateFactor(i, factor);
	}

	/* ************************************************************************* */
	template<class Factor>
	void FactorGraph<Factor>::push_back(const FactorGraph<Factor>& factors) {
		const_iterator factor = factors.begin();
		for (; factor != factors.end(); factor++)
			push_back(*factor);
	}

	/* ************************************************************************* */
	template<class Factor>
	void FactorGraph<Factor>::print(const string& s) const {
		cout << s << endl;
		printf("size: %d\n", (int) size());
		for (size_t i = 0; i < factors_.size(); i++) {
			stringstream ss;
			ss << "factor " << i << ":";
			if (factors_[i] != NULL) factors_[i]->print(ss.str());
		}
	}

	/* ************************************************************************* */
	template<class Factor>
	bool FactorGraph<Factor>::equals(const FactorGraph<Factor>& fg, double tol) const {
		/** check whether the two factor graphs have the same number of factors_ */
		if (factors_.size() != fg.size()) return false;

		/** check whether the factors_ are the same */
		for (size_t i = 0; i < factors_.size(); i++) {
			// TODO: Doesn't this force order of factor insertion?
			sharedFactor f1 = factors_[i], f2 = fg.factors_[i];
			if (f1 == NULL && f2 == NULL) continue;
			if (f1 == NULL || f2 == NULL) return false;
			if (!f1->equals(*f2, tol)) return false;
		}
		return true;
	}

	/* ************************************************************************* */
	template<class Factor>
	size_t FactorGraph<Factor>::nrFactors() const {
		size_t size_ = 0;
		for (const_iterator factor = factors_.begin(); factor != factors_.end(); factor++)
			if (*factor != NULL) size_++;
		return size_;
	}

	/* ************************************************************************* */
	template<class Factor>
	Ordering FactorGraph<Factor>::keys() const {
		Ordering keys;
		transform(indices_.begin(), indices_.end(), back_inserter(keys),
				_Select1st<Indices::value_type> ());
		return keys;
	}

	/* ************************************************************************* */
	/** O(1)                                                                     */
	/* ************************************************************************* */
	template<class Factor>
	list<size_t> FactorGraph<Factor>::factors(const Symbol& key) const {
		return indices_.at(key);
	}

	/* ************************************************************************* *
	 * Call colamd given a column-major symbolic matrix A
	 * @param n_col colamd arg 1: number of rows in A
	 * @param n_row colamd arg 2: number of columns in A
	 * @param nrNonZeros number of non-zero entries in A
	 * @param columns map from keys to a sparse column of non-zero row indices
	 * @param lastKeys set of keys that should appear last in the ordering
	 * ************************************************************************* */
	template<class Key>
	void colamd(int n_col, int n_row, int nrNonZeros,
			const map<Key, vector<int> >& columns, Ordering& ordering, const set<
					Symbol>& lastKeys) {

		// Convert to compressed column major format colamd wants it in (== MATLAB format!)
		int Alen = ccolamd_recommended(nrNonZeros, n_row, n_col); /* colamd arg 3: size of the array A */
		int * A = new int[Alen]; /* colamd arg 4: row indices of A, of size Alen */
		int * p = new int[n_col + 1]; /* colamd arg 5: column pointers of A, of size n_col+1 */
		int * cmember = new int[n_col]; /* Constraint set of A, of size n_col */

		p[0] = 0;
		int j = 1;
		int count = 0;
		typedef typename map<Key, vector<int> >::const_iterator iterator;
		bool front_exists = false;
		vector<Key> initialOrder;
		for (iterator it = columns.begin(); it != columns.end(); it++) {
			const Key& key = it->first;
			const vector<int>& column = it->second;
			initialOrder.push_back(key);
			BOOST_FOREACH(int i, column)
							A[count++] = i; // copy sparse column
			p[j] = count; // column j (base 1) goes from A[j-1] to A[j]-1
			if (lastKeys.find(key) == lastKeys.end()) {
				cmember[j - 1] = 0;
				front_exists = true;
			} else {
				cmember[j - 1] = 1; // force lastKeys to be at the end
			}
			j += 1;
		}
		if (!front_exists) { // if only 1 entries, set everything to 0...
			for (int j = 0; j < n_col; j++)
				cmember[j] = 0;
		}

		double* knobs = NULL; /* colamd arg 6: parameters (uses defaults if NULL) */
		int stats[CCOLAMD_STATS]; /* colamd arg 7: colamd output statistics and error codes */

		// call colamd, result will be in p *************************************************
		/* TODO: returns (1) if successful, (0) otherwise*/
		::ccolamd(n_row, n_col, Alen, A, p, knobs, stats, cmember);
		// **********************************************************************************
		delete[] A; // delete symbolic A
		delete[] cmember;

		// Convert elimination ordering in p to an ordering
		for (int j = 0; j < n_col; j++)
			ordering.push_back(initialOrder[p[j]]);
		delete[] p; // delete colamd result vector
	}

	/* ************************************************************************* */
	template<class Factor>
	void FactorGraph<Factor>::getOrdering(Ordering& ordering,
			const set<Symbol>& lastKeys,
			boost::optional<const set<Symbol>&> scope) const {

		// A factor graph is really laid out in row-major format, each factor a row
		// Below, we compute a symbolic matrix stored in sparse columns.
		map<Symbol, vector<int> > columns; // map from keys to a sparse column of non-zero row indices
		int nrNonZeros = 0; // number of non-zero entries
		int n_row = 0; /* colamd arg 1: number of rows in A */

		// loop over all factors = rows
		bool inserted;
		bool hasInterested = scope.is_initialized();
		BOOST_FOREACH(const sharedFactor& factor, factors_) {
				if (factor == NULL) continue;
				list<Symbol> keys = factor->keys();
				inserted = false;
				BOOST_FOREACH(const Symbol& key, keys) {
						if (!hasInterested || scope->find(key) != scope->end()) {
							columns[key].push_back(n_row);
							nrNonZeros++;
							inserted = true;
						}
					}
				if (inserted) n_row++;
			}
		int n_col = (int) (columns.size()); /* colamd arg 2: number of columns in A */
		if (n_col != 0) colamd(n_col, n_row, nrNonZeros, columns, ordering, lastKeys);
	}

	/* ************************************************************************* */
	template<class Factor>
	Ordering FactorGraph<Factor>::getOrdering() const {
		Ordering ordering;
		set<Symbol> lastKeys;
		getOrdering(ordering, lastKeys);
		return ordering;
	}

	/* ************************************************************************* */
	template<class Factor>
	boost::shared_ptr<Ordering> FactorGraph<Factor>::getOrdering_() const {
		boost::shared_ptr<Ordering> ordering(new Ordering);
		set<Symbol> lastKeys;
		getOrdering(*ordering, lastKeys);
		return ordering;
	}

	/* ************************************************************************* */
	template<class Factor>
	Ordering FactorGraph<Factor>::getOrdering(const set<Symbol>& scope) const {
		Ordering ordering;
		set<Symbol> lastKeys;
		getOrdering(ordering, lastKeys, scope);
		return ordering;
	}

	/* ************************************************************************* */
	template<class Factor>
	Ordering FactorGraph<Factor>::getConstrainedOrdering(
			const set<Symbol>& lastKeys) const {
		Ordering ordering;
		getOrdering(ordering, lastKeys);
		return ordering;
	}

	/* ************************************************************************* */
	template<class Factor> template<class Key, class Factor2>
	PredecessorMap<Key> FactorGraph<Factor>::findMinimumSpanningTree() const {

		SDGraph<Key> g = gtsam::toBoostGraph<FactorGraph<Factor> , Factor2, Key>(
				*this);

		// find minimum spanning tree
		vector<typename SDGraph<Key>::Vertex> p_map(boost::num_vertices(g));
		prim_minimum_spanning_tree(g, &p_map[0]);

		// convert edge to string pairs
		PredecessorMap<Key> tree;
		typename SDGraph<Key>::vertex_iterator itVertex = boost::vertices(g).first;
		typename vector<typename SDGraph<Key>::Vertex>::iterator vi;
		for (vi = p_map.begin(); vi != p_map.end(); itVertex++, vi++) {
			Key key = boost::get(boost::vertex_name, g, *itVertex);
			Key parent = boost::get(boost::vertex_name, g, *vi);
			tree.insert(key, parent);
		}

		return tree;
	}

	/* ************************************************************************* */
	template<class Factor> template<class Key, class Factor2>
	void FactorGraph<Factor>::split(const PredecessorMap<Key>& tree, FactorGraph<
			Factor>& Ab1, FactorGraph<Factor>& Ab2) const {

		BOOST_FOREACH(const sharedFactor& factor, factors_)
					{
						if (factor->keys().size() > 2) throw(invalid_argument(
								"split: only support factors with at most two keys"));

						if (factor->keys().size() == 1) {
							Ab1.push_back(factor);
							continue;
						}

						boost::shared_ptr<Factor2> factor2 = boost::dynamic_pointer_cast<
								Factor2>(factor);
						if (!factor2) continue;

						Key key1 = factor2->key1();
						Key key2 = factor2->key2();
						// if the tree contains the key
						if ((tree.find(key1) != tree.end()
								&& tree.find(key1)->second.compare(key2) == 0) || (tree.find(
								key2) != tree.end() && tree.find(key2)->second.compare(key1)
								== 0))
							Ab1.push_back(factor2);
						else
							Ab2.push_back(factor2);
					}
	}

	/* ************************************************************************* */
	template<class Factor>
	std::pair<FactorGraph<Factor> , FactorGraph<Factor> > FactorGraph<Factor>::splitMinimumSpanningTree() const {
		//	create an empty factor graph T (tree) and factor graph C (constraints)
		FactorGraph<Factor> T;
		FactorGraph<Factor> C;
		DSF<Symbol> dsf(keys());

		//	while G is nonempty and T is not yet spanning
		for (size_t i = 0; i < size(); i++) {
			const sharedFactor& f = factors_[i];

			// retrieve the labels of all the keys
			set<Symbol> labels;
			BOOST_FOREACH(const Symbol& key, f->keys())
							labels.insert(dsf.findSet(key));

			//	if that factor connects two different trees, then add it to T
			if (labels.size() > 1) {
				T.push_back(f);
				set<Symbol>::const_iterator it = labels.begin();
				Symbol root = *it;
				for (it++; it != labels.end(); it++)
					dsf = dsf.makeUnion(root, *it);
			} else
				//	otherwise add that factor to C
				C.push_back(f);
		}
		return make_pair(T, C);
	}

	/* ************************************************************************* */
	template<class Factor> void FactorGraph<Factor>::checkGraphConsistency() const {
		// Consistency check for debugging

		// Make sure each factor is listed in its variables index lists
		for (size_t i = 0; i < factors_.size(); i++) {
			if (factors_[i] == NULL) {
				cout << "** Warning: factor " << i << " is NULL" << endl;
			} else {
				// Get involved variables
				list<Symbol> keys = factors_[i]->keys();

				// Make sure each involved variable is listed as being associated with this factor
				BOOST_FOREACH(const Symbol& var, keys)
							{
								if (std::find(indices_.at(var).begin(), indices_.at(var).end(),
										i) == indices_.at(var).end()) cout
										<< "*** Factor graph inconsistency: " << (string) var
										<< " is not mapped to factor " << i << endl;
							}
			}
		}

		// Make sure each factor listed for a variable actually involves that variable
		BOOST_FOREACH(const SymbolMap<list<size_t> >::value_type& var, indices_)
					{
						BOOST_FOREACH(size_t i, var.second)
									{
										if (i >= factors_.size()) {
											cout << "*** Factor graph inconsistency: "
													<< (string) var.first << " lists factor " << i
													<< " but the graph does not contain this many factors."
													<< endl;
										}
										if (factors_[i] == NULL) {
											cout << "*** Factor graph inconsistency: "
													<< (string) var.first << " lists factor " << i
													<< " but this factor is set to NULL." << endl;
										}
										list<Symbol> keys = factors_[i]->keys();
										if (std::find(keys.begin(), keys.end(), var.first)
												== keys.end()) {
											cout << "*** Factor graph inconsistency: "
													<< (string) var.first << " lists factor " << i
													<< " but this factor does not involve this variable."
													<< endl;
										}
									}
					}
	}

	/* ************************************************************************* */
	template<class Factor>
	void FactorGraph<Factor>::replace(size_t index, sharedFactor factor) {
		if (index >= factors_.size()) throw invalid_argument(boost::str(
				boost::format("Factor graph does not contain a factor with index %d.")
						% index));
		//if(factors_[index] == NULL)
		//  throw invalid_argument(boost::str(boost::format(
		//      "Factor with index %d is NULL." % index)));

		if (factors_[index] != NULL) {
			// Remove this factor from its variables' index lists
			BOOST_FOREACH(const Symbol& key, factors_[index]->keys())
						{
							indices_.at(key).remove(index);
						}
		}

		// Replace the factor
		factors_[index] = factor;
		associateFactor(index, factor);
	}

	/* ************************************************************************* */
	/** find all non-NULL factors for a variable, then set factors to NULL       */
	/* ************************************************************************* */
	template<class Factor>
	vector<boost::shared_ptr<Factor> > FactorGraph<Factor>::findAndRemoveFactors(
			const Symbol& key) {

		// find all factor indices associated with the key
		Indices::const_iterator it = indices_.find(key);
		if (it == indices_.end()) throw std::invalid_argument(
				"FactorGraph::findAndRemoveFactors: key " + (string) key + " not found");
		const list<size_t>& factorsAssociatedWithKey = it->second;

		vector<sharedFactor> found;
		BOOST_FOREACH(const size_t& i, factorsAssociatedWithKey) {
				sharedFactor& fi = factors_.at(i); // throws exception !
				if (fi == NULL) continue; // skip NULL factors
				found.push_back(fi); // add to found
				fi.reset(); // set factor to NULL == remove(i)
			}

		indices_.erase(key);

		return found;
	}

	/* ************************************************************************* */
	template<class Factor>
	std::pair<FactorGraph<Factor> , set<Symbol> > FactorGraph<Factor>::removeSingletons() {
		FactorGraph<Factor> singletonGraph;
		set<Symbol> singletons;

		while (true) {
			// find all the singleton variables
			Ordering new_singletons;
			Symbol key;
			list<size_t> indices;
			BOOST_FOREACH(boost::tie(key, indices), indices_)
						{
							// find out the number of factors associated with the current key
							size_t numValidFactors = 0;
							BOOST_FOREACH(const size_t& i, indices)
											if (factors_[i] != NULL) numValidFactors++;

							if (numValidFactors == 1) {
								new_singletons.push_back(key);
								BOOST_FOREACH(const size_t& i, indices)
												if (factors_[i] != NULL) singletonGraph.push_back(
														factors_[i]);
							}
						}
			singletons.insert(new_singletons.begin(), new_singletons.end());

			BOOST_FOREACH(const Symbol& singleton, new_singletons)
							findAndRemoveFactors(singleton);

			// exit when there are no more singletons
			if (new_singletons.empty()) break;
		}

		return make_pair(singletonGraph, singletons);
	}

	/* ************************************************************************* */
	template<class Factor>
	FactorGraph<Factor> combine(const FactorGraph<Factor>& fg1,
			const FactorGraph<Factor>& fg2) {
		// create new linear factor graph equal to the first one
		FactorGraph<Factor> fg = fg1;

		// add the second factors_ in the graph
		fg.push_back(fg2);

		return fg;
	}

	/* ************************************************************************* */
	template<class Factor> boost::shared_ptr<Factor> removeAndCombineFactors(
			FactorGraph<Factor>& factorGraph, const Symbol& key) {

		// find and remove the factors associated with key
		vector<boost::shared_ptr<Factor> > found = factorGraph.findAndRemoveFactors(key);

		// make a vector out of them and create a new factor
		boost::shared_ptr<Factor> new_factor(new Factor(found));

		// return it
		return new_factor;
	}

	/* ************************************************************************* */
} // namespace gtsam

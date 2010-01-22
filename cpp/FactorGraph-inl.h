/**
 * @file   FactorGraph-inl.h
 * This is a template definition file, include it where needed (only!)
 * so that the appropriate code is generated and link errors avoided.
 * @brief  Factor Graph Base Class
 * @author Carlos Nieto
 * @author Frank Dellaert
 * @author Alireza Fathi
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
#include <colamd/colamd.h>
#include "Ordering.h"
#include "FactorGraph.h"
#include "graph-inl.h"

#define INSTANTIATE_FACTOR_GRAPH(F) \
  template class FactorGraph<F>; \
  /*template boost::shared_ptr<F> removeAndCombineFactors(FactorGraph<F>&, const std::string&);*/ \
  template FactorGraph<F> combine(const FactorGraph<F>&, const FactorGraph<F>&);


// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

using namespace std;

namespace gtsam {

/* ************************************************************************* */
template<class Factor>
template<class Conditional>
FactorGraph<Factor>::FactorGraph(const BayesNet<Conditional>& bayesNet)
{
	typename BayesNet<Conditional>::const_iterator it = bayesNet.begin();
	for(; it != bayesNet.end(); it++) {
		sharedFactor factor(new Factor(*it));
		push_back(factor);
	}
}

/* ************************************************************************* */
template<class Factor>
void FactorGraph<Factor>::print(const string& s) const {
	cout << s << endl;
	printf("size: %d\n", (int) size());
	for (int i = 0; i < factors_.size(); i++) {
		stringstream ss;
		ss << "factor " << i << ":";
		if (factors_[i] != NULL) factors_[i]->print(ss.str());
	}
}

/* ************************************************************************* */
template<class Factor>
bool FactorGraph<Factor>::equals
	(const FactorGraph<Factor>& fg, double tol) const {
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
	int size_ = 0;
	for (const_iterator factor = factors_.begin(); factor != factors_.end(); factor++)
		if (*factor != NULL) size_++;
	return size_;
}

/* ************************************************************************* */
template<class Factor>
void FactorGraph<Factor>::push_back(sharedFactor factor) {
	factors_.push_back(factor);                  // add the actual factor
	if (factor==NULL) return;

	int i = factors_.size() - 1;                 // index of factor
	associateFactor(i, factor);
}

/* ************************************************************************* */
template<class Factor>
void FactorGraph<Factor>::push_back(const FactorGraph<Factor>& factors) {
	const_iterator factor = factors.begin();
	for (; factor!= factors.end(); factor++)
		push_back(*factor);
}

/* ************************************************************************* */
template<class Factor>
void FactorGraph<Factor>::replace(int index, sharedFactor factor) {
  if(index >= factors_.size())
    throw invalid_argument(boost::str(boost::format(
        "Factor graph does not contain a factor with index %d.") % index));
  //if(factors_[index] == NULL)
  //  throw invalid_argument(boost::str(boost::format(
  //      "Factor with index %d is NULL." % index)));

  if(factors_[index] != NULL) {
    // Remove this factor from its variables' index lists
    BOOST_FOREACH(const Symbol& key, factor->keys()) {
    	indices_.at(key).remove(index);
    }
  }

  // Replace the factor
  factors_[index] = factor;
  associateFactor(index, factor);
}

/* ************************************************************************* */
template<class Factor>
Ordering FactorGraph<Factor>::keys() const {
	Ordering keys;
	transform(indices_.begin(), indices_.end(),
			back_inserter(keys), _Select1st<Indices::value_type>());
	return keys;
}

/* ************************************************************************* */
/**
 * Call colamd given a column-major symbolic matrix A
 * @param n_col colamd arg 1: number of rows in A
 * @param n_row colamd arg 2: number of columns in A
 * @param nrNonZeros number of non-zero entries in A
 * @param columns map from keys to a sparse column of non-zero row indices
 */
template <class Key>
boost::shared_ptr<Ordering> colamd(int n_col, int n_row, int nrNonZeros, const map<Key, vector<int> >& columns) {

	// Convert to compressed column major format colamd wants it in (== MATLAB format!)
	vector<Key> initialOrder;
	int Alen = nrNonZeros*30;     /* colamd arg 3: size of the array A TODO: use Tim's function ! */
	int * A = new int[Alen];      /* colamd arg 4: row indices of A, of size Alen */
	int * p = new int[n_col + 1]; /* colamd arg 5: column pointers of A, of size n_col+1 */

	p[0] = 0;
	int j = 1;
	int count = 0;
	typedef typename map<Key, vector<int> >::const_iterator iterator;
	for(iterator it = columns.begin(); it != columns.end(); it++) {
		const Key& key = it->first;
		const vector<int>& column = it->second;
		initialOrder.push_back(key);
		BOOST_FOREACH(int i, column) A[count++] = i; // copy sparse column
		p[j] = count; // column j (base 1) goes from A[j-1] to A[j]-1
		j+=1;
	}

	double* knobs = NULL;    /* colamd arg 6: parameters (uses defaults if NULL) */
	int stats[COLAMD_STATS]; /* colamd arg 7: colamd output statistics and error codes */

	// call colamd, result will be in p *************************************************
	/* TODO: returns (1) if successful, (0) otherwise*/
	::colamd(n_row, n_col, Alen, A, p, knobs, stats);
	// **********************************************************************************
	delete [] A; // delete symbolic A

	// Convert elimination ordering in p to an ordering
	boost::shared_ptr<Ordering> result(new Ordering);
	for(int j = 0; j < n_col; j++)
	  result->push_back(initialOrder[p[j]]);
	delete [] p; // delete colamd result vector

	return result;
}
/* ************************************************************************* */
template<class Factor>
boost::shared_ptr<Ordering> FactorGraph<Factor>::getOrdering_() const{

	// A factor graph is really laid out in row-major format, each factor a row
	// Below, we compute a symbolic matrix stored in sparse columns.
	map<Symbol, vector<int> > columns; // map from keys to a sparse column of non-zero row indices
	int nrNonZeros = 0;             // number of non-zero entries
	int n_row = factors_.size();    /* colamd arg 1: number of rows in A */

	// loop over all factors = rows
	for (int i = 0; i < n_row; i++) {
		if (factors_[i]==NULL) continue;
		list<Symbol> keys = factors_[i]->keys();
		BOOST_FOREACH(const Symbol& key, keys) columns[key].push_back(i);
		nrNonZeros+= keys.size();
	}
	int n_col = (int)(columns.size()); /* colamd arg 2: number of columns in A */

	if(n_col == 0)
		return boost::shared_ptr<Ordering>(new Ordering); // empty ordering
	else
		return colamd(n_col, n_row, nrNonZeros, columns);
}


/* ************************************************************************* */
template<class Factor>
Ordering FactorGraph<Factor>::getOrdering() const {
		return *getOrdering_(); // empty ordering
}

/* ************************************************************************* */
/** O(1)                                                                     */
/* ************************************************************************* */
template<class Factor>
list<int> FactorGraph<Factor>::factors(const Symbol& key) const {
	return indices_.at(key);
}

/* ************************************************************************* */
/** find all non-NULL factors for a variable, then set factors to NULL       */
/* ************************************************************************* */
template<class Factor>
vector<boost::shared_ptr<Factor> >
FactorGraph<Factor>::findAndRemoveFactors(const Symbol& key) {
	vector<sharedFactor> found;

	const list<int>& indices = indices_.at(key);
	BOOST_FOREACH(const int& i, indices) {
		if(factors_[i] == NULL) continue; // skip NULL factors
		found.push_back(factors_[i]);     // add to found
		remove(i);                        // set factor to NULL.
	}
	return found;
}

/* ************************************************************************* */
template<class Factor>
void FactorGraph<Factor>::associateFactor(int index, sharedFactor factor) {
  list<Symbol> keys = factor->keys();          // get keys for factor
  // rtodo: Can optimize factor->keys to return a const reference

  BOOST_FOREACH(const Symbol& key, keys){             // for each key push i onto list
      Indices::iterator it = indices_.find(key); // old list for that key (if exists)
      if (it==indices_.end()){                   // there's no list yet
          list<int> indices(1,index);                  // so make one
          indices_.insert(make_pair(key,indices)); // insert new indices into factorMap
      }
      else {
        // rtodo: what is going on with this pointer?
          list<int> *indices_ptr = &(it->second);  // get the list
          indices_ptr->push_back(index);               // add the index i to it
      }
  }
}

/* ************************************************************************* */
template<class Factor> template <class Key, class Factor2>
PredecessorMap<Key> FactorGraph<Factor>::findMinimumSpanningTree() const {

	SDGraph<Key> g = gtsam::toBoostGraph<FactorGraph<Factor>, Factor2, Key>(*this);

	// find minimum spanning tree
	vector<typename SDGraph<Key>::Vertex> p_map(boost::num_vertices(g));
	prim_minimum_spanning_tree(g, &p_map[0]);

	// convert edge to string pairs
	PredecessorMap<Key> tree;
	typename SDGraph<Key>::vertex_iterator itVertex = boost::vertices(g).first;
	typename vector<typename SDGraph<Key>::Vertex>::iterator vi;
	for (vi = p_map.begin(); vi!=p_map.end(); itVertex++, vi++) {
		Key key = boost::get(boost::vertex_name, g, *itVertex);
		Key parent = boost::get(boost::vertex_name, g, *vi);
		tree.insert(key, parent);
	}

	return tree;
}

template<class Factor> template <class Key, class Factor2>
void FactorGraph<Factor>::split(const PredecessorMap<Key>& tree, FactorGraph<Factor>& Ab1, FactorGraph<Factor>& Ab2) const{

	BOOST_FOREACH(sharedFactor factor, factors_){
		if (factor->keys().size() > 2)
			throw(invalid_argument("split: only support factors with at most two keys"));

		if (factor->keys().size() == 1) {
			Ab1.push_back(factor);
			continue;
		}

		boost::shared_ptr<Factor2> factor2 = boost::dynamic_pointer_cast<Factor2>(factor);
		if (!factor2) continue;

		Key key1 = factor2->key1();
		Key key2 = factor2->key2();
		// if the tree contains the key
		if (tree.find(key1) != tree.end() && tree.find(key1)->second.compare(key2) == 0 ||
				tree.find(key2) != tree.end() && tree.find(key2)->second.compare(key1) == 0)
			Ab1.push_back(factor2);
		else
			Ab2.push_back(factor2);
	}
}

/* ************************************************************************* */
/* find factors and remove them from the factor graph: O(n)                  */
/* ************************************************************************* */
template<class Factor> boost::shared_ptr<Factor>
removeAndCombineFactors(FactorGraph<Factor>& factorGraph, const Symbol& key)
{
	vector<boost::shared_ptr<Factor> > found = factorGraph.findAndRemoveFactors(key);
	boost::shared_ptr<Factor> new_factor(new Factor(found));
	return new_factor;
}

/* ************************************************************************* */
template<class Factor>
FactorGraph<Factor> combine(const FactorGraph<Factor>& fg1, const FactorGraph<Factor>& fg2) {
	// create new linear factor graph equal to the first one
	FactorGraph<Factor> fg = fg1;

	// add the second factors_ in the graph
	fg.push_back(fg2);

	return fg;
}
}

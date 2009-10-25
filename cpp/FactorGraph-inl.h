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
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <colamd/colamd.h>
#include "Ordering.h"
#include "FactorGraph.h"

using namespace std;

namespace gtsam {

/* ************************************************************************* */
template<class Factor, class Config>
void FactorGraph<Factor, Config>::print(const string& s) const {
	cout << s << endl;
	printf("size: %d\n", (int) size());
	for (int i = 0; i < factors_.size(); i++) {
		stringstream ss;
		ss << "factor " << i << ":";
		if (factors_[i] != NULL) factors_[i]->print(ss.str());
	}
}

/* ************************************************************************* */
template<class Factor, class Config>
bool FactorGraph<Factor, Config>::equals
	(const FactorGraph<Factor, Config>& fg, double tol) const {
	/** check whether the two factor graphs have the same number of factors_ */
	if (factors_.size() != fg.size()) return false;

	/** check whether the factors_ are the same */
	for (size_t i = 0; i < factors_.size(); i++) {
		// TODO: Doesn't this force order of factor insertion?
		shared_factor f1 = factors_[i], f2 = fg.factors_[i];
		if (f1 == NULL && f2 == NULL) continue;
		if (f1 == NULL || f2 == NULL) return false;
		if (!f1->equals(*f2, tol)) return false;
	}
	return true;
}

/* ************************************************************************* */
template<class Factor, class Config>
void FactorGraph<Factor,Config>::push_back(shared_factor factor) {
	factors_.push_back(factor);  // add the actual factor
	int i = factors_.size() - 1; // index of factor
	list<string> keys = factor->keys(); // get keys for factor
	BOOST_FOREACH(string key, keys){    // for each key push i onto list
		Indices::iterator it = indices_.find(key); // old list for that key (if exists)
		if (it==indices_.end()){  // there's no list yet, so make one
			list<int> indices(1, i);
			indices_.insert(pair<string, list<int> >(key, indices));  // insert new indices into factorMap
		}
		else{
			list<int> *indices_ptr;
			indices_ptr = &(it->second);
			indices_ptr->push_back(i);
		}
	}
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
Ordering colamd(int n_col, int n_row, int nrNonZeros, const map<Key, vector<int> >& columns) {

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
	Ordering result;
	for(int j = 0; j < n_col; j++)
		result.push_back(initialOrder[j]);
	delete [] p; // delete colamd result vector

	return result;
}

/* ************************************************************************* */
template<class Factor, class Config>
Ordering FactorGraph<Factor,Config>::getOrdering() const {

	// A factor graph is really laid out in row-major format, each factor a row
	// Below, we compute a symbolic matrix stored in sparse columns.
	typedef string Key;                  // default case  with string keys
	map<Key, vector<int> > columns; // map from keys to a sparse column of non-zero row indices
	int nrNonZeros = 0;             // number of non-zero entries
	int n_row = factors_.size();    /* colamd arg 1: number of rows in A */

	// loop over all factors = rows
	for (int i = 0; i < n_row; i++) {
		list<Key> keys = factors_[i]->keys();
		BOOST_FOREACH(Key key, keys) columns[key].push_back(i);
		nrNonZeros+= keys.size();
	}
	int n_col = (int)(columns.size()); /* colamd arg 2: number of columns in A */

	if(n_col == 0)
		return Ordering(); // empty ordering
	else
		return colamd(n_col, n_row, nrNonZeros, columns);
}

/* ************************************************************************* */
}

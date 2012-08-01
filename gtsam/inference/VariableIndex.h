/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VariableIndex.h
 * @author  Richard Roberts
 * @date    Sep 12, 2010
 */

#pragma once

#include <vector>
#include <deque>
#include <stdexcept>
#include <boost/foreach.hpp>

#include <gtsam/base/FastList.h>
#include <gtsam/base/types.h>

namespace gtsam {

	class Permutation;

/**
 * The VariableIndex class computes and stores the block column structure of a
 * factor graph.  The factor graph stores a collection of factors, each of 
 * which involves a set of variables.  In contrast, the VariableIndex is built
 * from a factor graph prior to elimination, and stores the list of factors
 * that involve each variable.  This information is stored as a deque of
 * lists of factor indices.
 * \nosubgrouping
 */
class VariableIndex {
public:

  typedef boost::shared_ptr<VariableIndex> shared_ptr;
  typedef FastList<size_t> Factors;
  typedef Factors::iterator Factor_iterator;
  typedef Factors::const_iterator Factor_const_iterator;

protected:
  std::deque<Factors> index_;
  size_t nFactors_; // Number of factors in the original factor graph.
  size_t nEntries_; // Sum of involved variable counts of each factor.

public:

	/// @name Standard Constructors
	/// @{

  /** Default constructor, creates an empty VariableIndex */
  VariableIndex() : nFactors_(0), nEntries_(0) {}

  /**
   * Create a VariableIndex that computes and stores the block column structure
   * of a factor graph.  This constructor is used when the number of variables
   * is known beforehand.
   */
  template<class FactorGraph> VariableIndex(const FactorGraph& factorGraph, Index nVariables);

  /**
   * Create a VariableIndex that computes and stores the block column structure
   * of a factor graph.
   */
  template<class FactorGraph> VariableIndex(const FactorGraph& factorGraph);

	/// @}
	/// @name Standard Interface
	/// @{

  /**
   * The number of variable entries.  This is one greater than the variable
   * with the highest index.
   */
  Index size() const { return index_.size(); }

  /** The number of factors in the original factor graph */
  size_t nFactors() const { return nFactors_; }

  /** The number of nonzero blocks, i.e. the number of variable-factor entries */
  size_t nEntries() const { return nEntries_; }

  /** Access a list of factors by variable */
  const Factors& operator[](Index variable) const { checkVar(variable); return index_[variable]; }

	/// @}
	/// @name Testable
	/// @{

  /** Test for equality (for unit tests and debug assertions). */
  bool equals(const VariableIndex& other, double tol=0.0) const;

  /** Print the variable index (for unit tests and debugging). */
  void print(const std::string& str = "VariableIndex: ") const;

  /**
   * Output dual hypergraph to Metis file format for use with hmetis
   * In the dual graph, variables are hyperedges, factors are nodes.
   */
  void outputMetisFormat(std::ostream& os) const;


	/// @}
	/// @name Advanced Interface
	/// @{

  /**
   * Augment the variable index with new factors.  This can be used when
   * solving problems incrementally.
   */
  template<class FactorGraph> void augment(const FactorGraph& factors);

  /**
   * Remove entries corresponding to the specified factors.
	 * NOTE: We intentionally do not decrement nFactors_ because the factor
	 * indices need to remain consistent.  Removing factors from a factor graph
	 * does not shift the indices of other factors.  Also, we keep nFactors_
	 * one greater than the highest-numbered factor referenced in a VariableIndex.
	 *
   * @param indices The indices of the factors to remove, which must match \c factors
   * @param factors The factors being removed, which must symbolically correspond
   * exactly to the factors with the specified \c indices that were added.
   */
  template<typename CONTAINER, class FactorGraph> void remove(const CONTAINER& indices, const FactorGraph& factors);

	/// Permute the variables in the VariableIndex according to the given permutation
	void permuteInPlace(const Permutation& permutation);

	/** Remove unused empty variables at the end of the ordering (in debug mode
	 * verifies they are empty).
	 * @param nToRemove The number of unused variables at the end to remove
	 */
	void removeUnusedAtEnd(size_t nToRemove);

protected:
  Factor_iterator factorsBegin(Index variable) { checkVar(variable); return index_[variable].begin(); }	  ///<TODO: comment
  Factor_iterator factorsEnd(Index variable) { checkVar(variable); return index_[variable].end(); }				///<TODO: comment

  Factor_const_iterator factorsBegin(Index variable) const { checkVar(variable); return index_[variable].begin(); }	///<TODO: comment
  Factor_const_iterator factorsEnd(Index variable) const { checkVar(variable); return index_[variable].end(); }			///<TODO: comment

  /// Internal constructor to allocate a VariableIndex of the requested size
  VariableIndex(size_t nVars) : index_(nVars), nFactors_(0), nEntries_(0) {}

  /// Internal check of the validity of a variable
  void checkVar(Index variable) const { assert(variable < index_.size()); }

  /// Internal function to populate the variable index from a factor graph
  template<class FactorGraph> void fill(const FactorGraph& factorGraph);

	/// @}
};

/* ************************************************************************* */
template<class FactorGraph>
void VariableIndex::fill(const FactorGraph& factorGraph) {

  // Build index mapping from variable id to factor index
  for(size_t fi=0; fi<factorGraph.size(); ++fi) {
    if(factorGraph[fi]) {
      BOOST_FOREACH(const Index key, factorGraph[fi]->keys()) {
        if(key < index_.size()) {
          index_[key].push_back(fi);
          ++ nEntries_;
        }
      }
    }
    ++ nFactors_; // Increment factor count even if factors are null, to keep indices consistent
  }
}

/* ************************************************************************* */
template<class FactorGraph>
VariableIndex::VariableIndex(const FactorGraph& factorGraph) :
    nFactors_(0), nEntries_(0) {

  // If the factor graph is empty, return an empty index because inside this
  // if block we assume at least one factor.
  if(factorGraph.size() > 0) {
    // Find highest-numbered variable
    Index maxVar = 0;
    BOOST_FOREACH(const typename FactorGraph::sharedFactor& factor, factorGraph) {
      if(factor) {
        BOOST_FOREACH(const Index key, factor->keys()) {
          if(key > maxVar)
            maxVar = key;
        }
      }
    }

    // Allocate array
    index_.resize(maxVar+1);

    fill(factorGraph);
  }
}

/* ************************************************************************* */
template<class FactorGraph>
VariableIndex::VariableIndex(const FactorGraph& factorGraph, Index nVariables) :
    index_(nVariables), nFactors_(0), nEntries_(0) {
  fill(factorGraph);
}

/* ************************************************************************* */
template<class FactorGraph>
void VariableIndex::augment(const FactorGraph& factors) {
  // If the factor graph is empty, return an empty index because inside this
  // if block we assume at least one factor.
  if(factors.size() > 0) {
    // Find highest-numbered variable
    Index maxVar = 0;
    BOOST_FOREACH(const typename FactorGraph::sharedFactor& factor, factors) {
      if(factor) {
        BOOST_FOREACH(const Index key, factor->keys()) {
          if(key > maxVar)
            maxVar = key;
        }
      }
    }

    // Allocate index
    index_.resize(std::max(index_.size(), maxVar+1));

    // Augment index mapping from variable id to factor index
    size_t orignFactors = nFactors_;
    for(size_t fi=0; fi<factors.size(); ++fi) {
      if(factors[fi]) {
        BOOST_FOREACH(const Index key, factors[fi]->keys()) {
          index_[key].push_back(orignFactors + fi);
          ++ nEntries_;
        }
      }
      ++ nFactors_; // Increment factor count even if factors are null, to keep indices consistent
    }
  }
}

/* ************************************************************************* */
template<typename CONTAINER, class FactorGraph>
void VariableIndex::remove(const CONTAINER& indices, const FactorGraph& factors) {
	// NOTE: We intentionally do not decrement nFactors_ because the factor
	// indices need to remain consistent.  Removing factors from a factor graph
	// does not shift the indices of other factors.  Also, we keep nFactors_
	// one greater than the highest-numbered factor referenced in a VariableIndex.
  for(size_t fi=0; fi<factors.size(); ++fi)
    if(factors[fi]) {
      for(size_t ji = 0; ji < factors[fi]->keys().size(); ++ji) {
        Factors& factorEntries = index_[factors[fi]->keys()[ji]];
        Factors::iterator entry = std::find(factorEntries.begin(), factorEntries.end(), indices[fi]);
        if(entry == factorEntries.end())
          throw std::invalid_argument("Internal error, indices and factors passed into VariableIndex::remove are not consistent with the existing variable index");
        factorEntries.erase(entry);
        -- nEntries_;
      }
    }
}

}

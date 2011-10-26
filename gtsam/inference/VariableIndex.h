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

#include <gtsam/base/FastList.h>
#include <gtsam/inference/Permutation.h>

#include <vector>
#include <boost/foreach.hpp>

namespace gtsam {

class Inference;

/**
 * The VariableIndex class computes and stores the block column structure of a
 * factor graph.  The factor graph stores a collection of factors, each of 
 * which involves a set of variables.  In contrast, the VariableIndex is built
 * from a factor graph prior to elimination, and stores the list of factors
 * that involve each variable.  This information is stored as a vector of
 * lists of factor indices.
 */
class VariableIndex {
public:

  typedef boost::shared_ptr<VariableIndex> shared_ptr;
  typedef FastList<size_t> Factors;
  typedef Factors::iterator Factor_iterator;
  typedef Factors::const_iterator Factor_const_iterator;

protected:
  std::vector<Factors> indexUnpermuted_;
  Permuted<std::vector<Factors> > index_; // Permuted view of indexUnpermuted.
  size_t nFactors_; // Number of factors in the original factor graph.
  size_t nEntries_; // Sum of involved variable counts of each factor.

public:
  /** Default constructor, creates an empty VariableIndex */
  VariableIndex() : index_(indexUnpermuted_), nFactors_(0), nEntries_(0) {}

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

  /** Access a list of factors by variable */
  Factors& operator[](Index variable) { checkVar(variable); return index_[variable]; }

  /**
   * Apply a variable permutation.  Does not rearrange data, just permutes
   * future lookups by variable.
   */
  void permute(const Permutation& permutation);

  /**
   * Augment the variable index with new factors.  This can be used when
   * solving problems incrementally.
   */
  template<class FactorGraph> void augment(const FactorGraph& factorGraph);

  /** Test for equality (for unit tests and debug assertions). */
  bool equals(const VariableIndex& other, double tol=0.0) const;

  /** Print the variable index (for unit tests and debugging). */
  void print(const std::string& str = "VariableIndex: ") const;

protected:
  VariableIndex(size_t nVars) : indexUnpermuted_(nVars), index_(indexUnpermuted_), nFactors_(0), nEntries_(0) {}
  void checkVar(Index variable) const { assert(variable < index_.size()); }

  template<class FactorGraph> void fill(const FactorGraph& factorGraph);

  Factor_iterator factorsBegin(Index variable) { checkVar(variable); return index_[variable].begin(); }
  Factor_const_iterator factorsBegin(Index variable) const { checkVar(variable); return index_[variable].begin(); }
  Factor_iterator factorsEnd(Index variable) { checkVar(variable); return index_[variable].end(); }
  Factor_const_iterator factorsEnd(Index variable) const { checkVar(variable); return index_[variable].end(); }
};

/* ************************************************************************* */
template<class FactorGraph>
void VariableIndex::fill(const FactorGraph& factorGraph) {

  // Build index mapping from variable id to factor index
  for(size_t fi=0; fi<factorGraph.size(); ++fi)
    if(factorGraph[fi]) {
      BOOST_FOREACH(const Index key, factorGraph[fi]->keys()) {
        if(key < index_.size()) {
          index_[key].push_back(fi);
          ++ nEntries_;
        }
      }
      ++ nFactors_;
    }
}

/* ************************************************************************* */
template<class FactorGraph>
VariableIndex::VariableIndex(const FactorGraph& factorGraph) :
    index_(indexUnpermuted_), nFactors_(0), nEntries_(0) {

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
    index_.container().resize(maxVar+1);
    index_.permutation() = Permutation::Identity(maxVar+1);

    fill(factorGraph);
  }
}

/* ************************************************************************* */
template<class FactorGraph>
VariableIndex::VariableIndex(const FactorGraph& factorGraph, Index nVariables) :
    indexUnpermuted_(nVariables), index_(indexUnpermuted_), nFactors_(0), nEntries_(0) {
  fill(factorGraph);
}

/* ************************************************************************* */
template<class FactorGraph>
void VariableIndex::augment(const FactorGraph& factorGraph) {
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

    // Allocate index
    Index originalSize = index_.size();
    index_.container().resize(std::max(index_.size(), maxVar+1));
    index_.permutation().resize(index_.container().size());
    for(Index var=originalSize; var<index_.permutation().size(); ++var)
      index_.permutation()[var] = var;

    // Augment index mapping from variable id to factor index
    size_t orignFactors = nFactors_;
    for(size_t fi=0; fi<factorGraph.size(); ++fi)
      if(factorGraph[fi]) {
        BOOST_FOREACH(const Index key, factorGraph[fi]->keys()) {
          index_[key].push_back(orignFactors + fi);
          ++ nEntries_;
        }
        ++ nFactors_;
      }
  }
}

}

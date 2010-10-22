/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VariableIndex.h
 * @brief   
 * @author  Richard Roberts
 * @created Sep 12, 2010
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/base/FastList.h>
#include <gtsam/inference/Permutation.h>

#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

namespace gtsam {

class Inference;

/**
 * The VariableIndex class stores the information necessary to perform
 * elimination, including an index of factors involving each variable and
 * the structure of the intermediate joint factors that develop during
 * elimination.  This maps variables to collections of factor indices.
 */
class VariableIndex {
public:

  typedef boost::shared_ptr<VariableIndex> shared_ptr;
  typedef FastList<size_t> Factors;
  typedef typename Factors::iterator Factor_iterator;
  typedef typename Factors::const_iterator Factor_const_iterator;

protected:
  std::vector<Factors> indexUnpermuted_;
  Permuted<std::vector<Factors>, Factors&> index_;
  size_t nFactors_;
  size_t nEntries_; // Sum of involved variable counts of each factor

public:
  VariableIndex() : index_(indexUnpermuted_), nFactors_(0), nEntries_(0) {}
  template<class FactorGraph> VariableIndex(const FactorGraph& factorGraph, Index nVariables);
  template<class FactorGraph> VariableIndex(const FactorGraph& factorGraph);

  Index size() const { return index_.size(); }
  size_t nFactors() const { return nFactors_; }
  size_t nEntries() const { return nEntries_; }
  const Factors& operator[](Index variable) const { checkVar(variable); return index_[variable]; }
  Factors& operator[](Index variable) { checkVar(variable); return index_[variable]; }
  void permute(const Permutation& permutation);
  template<class FactorGraph> void augment(const FactorGraph& factorGraph);
  void rebaseFactors(ptrdiff_t baseIndexChange);

  bool equals(const VariableIndex& other, double tol=0.0) const;
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

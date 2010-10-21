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
#include <gtsam/inference/Permutation.h>

#include <vector>
#include <deque>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

namespace gtsam {

class Inference;

/**
 * The VariableIndex can internally use either a vector or a deque to store
 * variables.  For one-shot elimination, using a vector will give the best
 * performance, and this is the default.  For incremental updates, such as in
 * ISAM and ISAM2, deque storage is used to prevent frequent reallocation.
 */
struct VariableIndexStorage_vector { template<typename T> struct type_factory { typedef std::vector<T> type; }; };
struct VariableIndexStorage_deque { template<typename T> struct type_factory { typedef std::deque<T> type; }; };

/**
 * The VariableIndex class stores the information necessary to perform
 * elimination, including an index of factors involving each variable and
 * the structure of the intermediate joint factors that develop during
 * elimination.  This maps variables to deque's of pair<size_t,size_t>,
 * which is pairs the factor index with the position of the variable within
 * the factor.
 */
struct _mapped_factor_type {
  size_t factorIndex;
  size_t variablePosition;
  _mapped_factor_type(size_t _factorIndex, size_t _variablePosition) : factorIndex(_factorIndex), variablePosition(_variablePosition) {}
  bool operator==(const _mapped_factor_type& o) const { return factorIndex == o.factorIndex && variablePosition == o.variablePosition; }
};
template<class VARIABLEINDEXSTORAGE=VariableIndexStorage_vector>
class VariableIndex {
public:

  typedef _mapped_factor_type mapped_factor_type;
  typedef boost::shared_ptr<VariableIndex> shared_ptr;
  typedef std::deque<mapped_factor_type> mapped_type;
  typedef typename mapped_type::iterator factor_iterator;
  typedef typename mapped_type::const_iterator const_factor_iterator;

protected:
  typedef typename VARIABLEINDEXSTORAGE::template type_factory<mapped_type>::type storage_type;
  storage_type indexUnpermuted_;
  Permuted<storage_type, typename storage_type::value_type&> index_;
  size_t nFactors_;
  size_t nEntries_; // Sum of involved variable counts of each factor

public:
  VariableIndex() : index_(indexUnpermuted_), nFactors_(0), nEntries_(0) {}
  template<class FactorGraph> VariableIndex(const FactorGraph& factorGraph, Index nVariables);
  template<class FactorGraph> VariableIndex(const FactorGraph& factorGraph);

  Index size() const { return index_.size(); }
  size_t nFactors() const { return nFactors_; }
  size_t nEntries() const { return nEntries_; }
  const mapped_type& operator[](Index variable) const { checkVar(variable); return index_[variable]; }
  mapped_type& operator[](Index variable) { checkVar(variable); return index_[variable]; }
  void permute(const Permutation& permutation);
  template<class FactorGraph> void augment(const FactorGraph& factorGraph);
  void rebaseFactors(ptrdiff_t baseIndexChange);

  template<class Derived> bool equals(const Derived& other, double tol=0.0) const;
  void print(const std::string& str = "VariableIndex: ") const;

protected:
  VariableIndex(size_t nVars) : indexUnpermuted_(nVars), index_(indexUnpermuted_), nFactors_(0), nEntries_(0) {}
  void checkVar(Index variable) const { assert(variable < index_.size()); }

  template<class FactorGraph> void fill(const FactorGraph& factorGraph);

  factor_iterator factorsBegin(Index variable) { checkVar(variable); return index_[variable].begin(); }
  const_factor_iterator factorsBegin(Index variable) const { checkVar(variable); return index_[variable].begin(); }
  factor_iterator factorsEnd(Index variable) { checkVar(variable); return index_[variable].end(); }
  const_factor_iterator factorsEnd(Index variable) const { checkVar(variable); return index_[variable].end(); }

  friend class Inference;
};

/* ************************************************************************* */
template<class Storage>
void VariableIndex<Storage>::permute(const Permutation& permutation) {
#ifndef NDEBUG
  // Assert that the permutation does not leave behind any non-empty variables,
  // otherwise the nFactors and nEntries counts would be incorrect.
  for(Index j=0; j<this->index_.size(); ++j)
    if(find(permutation.begin(), permutation.end(), j) == permutation.end())
      assert(this->operator[](j).empty());
#endif
  index_.permute(permutation);
//  storage_type original(this->index_.size());
//  this->index_.swap(original);
//  for(Index j=0; j<permutation.size(); ++j)
//    this->index_[j].swap(original[permutation[j]]);
}

/* ************************************************************************* */
template<class Storage>
template<class FactorGraph>
void VariableIndex<Storage>::fill(const FactorGraph& factorGraph) {

  // Build index mapping from variable id to factor index
  for(size_t fi=0; fi<factorGraph.size(); ++fi)
    if(factorGraph[fi]) {
      Index fvari = 0;
      BOOST_FOREACH(const Index key, factorGraph[fi]->keys()) {
        if(key < index_.size()) {
          index_[key].push_back(mapped_factor_type(fi, fvari));
          ++ fvari;
          ++ nEntries_;
        }
      }
      ++ nFactors_;
    }

}

/* ************************************************************************* */
template<class Storage>
template<class FactorGraph>
VariableIndex<Storage>::VariableIndex(const FactorGraph& factorGraph) :
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
template<class Storage>
template<class FactorGraph>
VariableIndex<Storage>::VariableIndex(const FactorGraph& factorGraph, Index nVariables) :
    indexUnpermuted_(nVariables), index_(indexUnpermuted_), nFactors_(0), nEntries_(0) {
  fill(factorGraph);
}

/* ************************************************************************* */
template<class Storage>
template<class FactorGraph>
void VariableIndex<Storage>::augment(const FactorGraph& factorGraph) {
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
        Index fvari = 0;
        BOOST_FOREACH(const Index key, factorGraph[fi]->keys()) {
          index_[key].push_back(mapped_factor_type(orignFactors + fi, fvari));
          ++ fvari;
          ++ nEntries_;
        }
        ++ nFactors_;
      }
  }
}

/* ************************************************************************* */
template<class Storage>
void VariableIndex<Storage>::rebaseFactors(ptrdiff_t baseIndexChange) {
  BOOST_FOREACH(mapped_type& factors, index_.container()) {
    BOOST_FOREACH(mapped_factor_type& factor, factors) {
      factor.factorIndex += baseIndexChange;
    }
  }
}

/* ************************************************************************* */
template<class Storage>
template<class Derived>
bool VariableIndex<Storage>::equals(const Derived& other, double tol) const {
  if(this->nEntries_ == other.nEntries_ && this->nFactors_ == other.nFactors_) {
    for(size_t var=0; var < std::max(this->index_.size(), other.index_.size()); ++var)
      if(var >= this->index_.size() || var >= other.index_.size()) {
        if(!((var >= this->index_.size() && other.index_[var].empty()) ||
            (var >= other.index_.size() && this->index_[var].empty())))
          return false;
      } else if(this->index_[var] != other.index_[var])
        return false;
  } else
    return false;
  return true;
}

/* ************************************************************************* */
template<class Storage>
void VariableIndex<Storage>::print(const std::string& str) const {
  std::cout << str;
  Index var = 0;
  BOOST_FOREACH(const mapped_type& variable, index_.container()) {
    Permutation::const_iterator rvar = find(index_.permutation().begin(), index_.permutation().end(), var);
    assert(rvar != index_.permutation().end());
    std::cout << "var " << (rvar-index_.permutation().begin()) << ":";
    BOOST_FOREACH(const mapped_factor_type& factor, variable) {
      std::cout << " " << factor.factorIndex << "-" << factor.variablePosition;
    }
    std::cout << "\n";
    ++ var;
  }
  std::cout << std::flush;
}

}

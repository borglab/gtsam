/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   BayesNet-inl.h
 * @brief  Bayes net template definitions
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Testable.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/format.hpp>

#include <boost/assign/std/vector.hpp> // for +=
using boost::assign::operator+=;

#include <algorithm>
#include <iostream>
#include <fstream>

namespace gtsam {

	/* ************************************************************************* */
	template<class CONDITIONAL>
	void BayesNet<CONDITIONAL>::print(const std::string& s,
			const IndexFormatter& formatter) const {
		std::cout << s;
		BOOST_REVERSE_FOREACH(sharedConditional conditional, conditionals_)
			conditional->print("Conditional", formatter);
	}

  /* ************************************************************************* */
  template<class CONDITIONAL>
  void BayesNet<CONDITIONAL>::printStats(const std::string& s) const {

    const size_t n = conditionals_.size();
    size_t max_size = 0;
    size_t total = 0;
    BOOST_REVERSE_FOREACH(sharedConditional conditional, conditionals_) {
      max_size = std::max(max_size, conditional->size());
      total += conditional->size();
    }
    std::cout << s
              << "maximum conditional size = " << max_size << std::endl
              << "average conditional size = " << total / n << std::endl
              << "density = " << 100.0 * total / (double) (n*(n+1)/2) << " %" << std::endl;
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  bool BayesNet<CONDITIONAL>::equals(const BayesNet& cbn, double tol) const {
    if (size() != cbn.size())
      return false;
    return std::equal(conditionals_.begin(), conditionals_.end(),
        cbn.conditionals_.begin(), equals_star<CONDITIONAL>(tol));
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  typename BayesNet<CONDITIONAL>::const_iterator BayesNet<CONDITIONAL>::find(
      Index key) const {
    for (const_iterator it = begin(); it != end(); ++it)
      if (std::find((*it)->beginFrontals(), (*it)->endFrontals(), key)
          != (*it)->endFrontals())
        return it;
    return end();
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  typename BayesNet<CONDITIONAL>::iterator BayesNet<CONDITIONAL>::find(
      Index key) {
    for (iterator it = begin(); it != end(); ++it)
      if (std::find((*it)->beginFrontals(), (*it)->endFrontals(), key)
          != (*it)->endFrontals())
        return it;
    return end();
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  void BayesNet<CONDITIONAL>::permuteWithInverse(
      const Permutation& inversePermutation) {
    BOOST_FOREACH(sharedConditional conditional, conditionals_) {
      conditional->permuteWithInverse(inversePermutation);
    }
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  bool BayesNet<CONDITIONAL>::permuteSeparatorWithInverse(
      const Permutation& inversePermutation) {
    bool separatorChanged = false;
    BOOST_FOREACH(sharedConditional conditional, conditionals_) {
      if (conditional->permuteSeparatorWithInverse(inversePermutation))
        separatorChanged = true;
    }
    return separatorChanged;
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  void BayesNet<CONDITIONAL>::push_back(const BayesNet<CONDITIONAL> bn) {
    BOOST_FOREACH(sharedConditional conditional,bn.conditionals_)
      push_back(conditional);
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  void BayesNet<CONDITIONAL>::push_front(const BayesNet<CONDITIONAL> bn) {
    BOOST_FOREACH(sharedConditional conditional,bn.conditionals_)
      push_front(conditional);
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  void BayesNet<CONDITIONAL>::popLeaf(iterator conditional) {
#ifndef NDEBUG
    BOOST_FOREACH(typename CONDITIONAL::shared_ptr checkConditional, conditionals_) {
      BOOST_FOREACH(Index key, (*conditional)->frontals()) {
        if(std::find(checkConditional->beginParents(), checkConditional->endParents(), key) != checkConditional->endParents())
        throw std::invalid_argument(
            "Debug mode exception:  in BayesNet::popLeaf, the requested conditional is not a leaf.");
      }
    }
#endif
    conditionals_.erase(conditional);
  }

  /* ************************************************************************* */
  template<class CONDITIONAL>
  FastList<Index> BayesNet<CONDITIONAL>::ordering() const {
    FastList<Index> ord;
    BOOST_FOREACH(sharedConditional conditional,conditionals_)
      ord.insert(ord.begin(), conditional->beginFrontals(),
          conditional->endFrontals());
    return ord;
  }

//	/* ************************************************************************* */
//	template<class CONDITIONAL>
//	void BayesNet<CONDITIONAL>::saveGraph(const std::string &s) const {
//		ofstream of(s.c_str());
//		of<< "digraph G{\n";
//		BOOST_FOREACH(const_sharedConditional conditional,conditionals_) {
//			Index child = conditional->key();
//			BOOST_FOREACH(Index parent, conditional->parents()) {
//				of << parent << "->" << child << endl;
//			}
//		}
//		of<<"}";
//		of.close();
//	}
//
  /* ************************************************************************* */

  template<class CONDITIONAL>
  typename BayesNet<CONDITIONAL>::sharedConditional BayesNet<CONDITIONAL>::operator[](
      Index key) const {
    BOOST_FOREACH(typename CONDITIONAL::shared_ptr conditional, conditionals_) {
      typename CONDITIONAL::const_iterator it = std::find(
          conditional->beginFrontals(), conditional->endFrontals(), key);
      if (it != conditional->endFrontals()) {
        return conditional;
      }
    }
    throw(std::invalid_argument(
        (boost::format("BayesNet::operator['%1%']: not found") % key).str()));
  }

/* ************************************************************************* */

} // namespace gtsam

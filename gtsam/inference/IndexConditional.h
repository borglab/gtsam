/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexConditional.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 17, 2010
 */

#pragma once

#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/IndexFactor.h>

namespace gtsam {

  /**
   * IndexConditional serves two purposes.  It is the base class for
   * GaussianConditional, and also functions as a symbolic conditional with
   * Index keys, produced by symbolic elimination of IndexFactor.
   *
   * It derives from Conditional with a key type of Index, which is an
   * unsigned integer.
   */
  class IndexConditional : public Conditional<Index> {

  public:

    typedef IndexConditional This;
    typedef Conditional<Index> Base;
    typedef IndexFactor FactorType;
    typedef boost::shared_ptr<IndexConditional> shared_ptr;

    /** Empty Constructor to make serialization possible */
    IndexConditional() {}

    /** No parents */
    IndexConditional(Index j) : Base(j) {}

    /** Single parent */
    IndexConditional(Index j, Index parent) : Base(j, parent) {}

    /** Two parents */
    IndexConditional(Index j, Index parent1, Index parent2) : Base(j, parent1, parent2) {}

    /** Three parents */
    IndexConditional(Index j, Index parent1, Index parent2, Index parent3) : Base(j, parent1, parent2, parent3) {}

    /** Constructor from a frontal variable and a vector of parents */
    IndexConditional(Index j, const std::vector<Index>& parents) : Base(j, parents) {}

    /** Constructor from a frontal variable and an iterator range of parents */
    template<typename ITERATOR>
    static shared_ptr FromRange(Index j, ITERATOR firstParent, ITERATOR lastParent) {
      return Base::FromRange<This>(j, firstParent, lastParent); }

    /** Named constructor from any number of frontal variables and parents */
    template<typename ITERATOR>
    static shared_ptr FromRange(ITERATOR firstKey, ITERATOR lastKey, size_t nrFrontals) {
      return Base::FromRange<This>(firstKey, lastKey, nrFrontals); }

    /** Convert to a factor */
    IndexFactor::shared_ptr toFactor() const { return IndexFactor::shared_ptr(new IndexFactor(*this)); }

  };

}

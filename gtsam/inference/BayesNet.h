/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file    BayesNet.h
* @brief   Bayes network
* @author  Frank Dellaert
* @author  Richard Roberts
*/

#pragma once

#include <boost/shared_ptr.hpp>

#include <gtsam/inference/FactorGraph.h>

namespace gtsam {

  /**
  * A BayesNet is a tree of conditionals, stored in elimination order.
  *
  * todo:  how to handle Bayes nets with an optimize function?  Currently using global functions.
  * \nosubgrouping
  */
  template<class CONDITIONAL>
  class BayesNet : public FactorGraph<CONDITIONAL> {

  private:

    typedef FactorGraph<CONDITIONAL> Base;

  public:
    typedef typename boost::shared_ptr<CONDITIONAL> sharedConditional; ///< A shared pointer to a conditional

  protected:
    /// @name Standard Constructors
    /// @{

    /** Default constructor as an empty BayesNet */
    BayesNet() {};

    /** Construct from iterator over conditionals */
    template<typename ITERATOR>
    BayesNet(ITERATOR firstConditional, ITERATOR lastConditional) : Base(firstConditional, lastConditional) {}

    /// @}

  public:
    /// @name Testable
    /// @{

    /** print out graph */
    void print(const std::string& s = "BayesNet",
      const KeyFormatter& formatter = DefaultKeyFormatter) const;

    /// @}

    /// @name Standard Interface
    /// @{

    void saveGraph(const std::string &s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
  };

}

#include <gtsam/inference/BayesNet-inst.h>

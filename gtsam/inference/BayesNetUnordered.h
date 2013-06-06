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

namespace gtsam {

  /**
  * A BayesNet is a tree of conditionals, stored in elimination order.
  *
  * todo:  how to handle Bayes nets with an optimize function?  Currently using global functions.
  * \nosubgrouping
  */
  template<class CONDITIONAL>
  class BayesNetUnordered {

  public:

    typedef typename boost::shared_ptr<CONDITIONAL> sharedConditional; ///< A shared pointer to a conditional

    /// Internal tree node that stores the conditional and the elimination parent
    struct Node {
      sharedConditional conditional;
      boost::shared_ptr<Node> parent;
    };

    typedef boost::shared_ptr<Node> sharedNode; ///< A shared pointer to a node (used internally)

  protected:

    sharedNode roots_; ///< Tree roots

  public:

    /// @name Standard Constructors
    /// @{

    /** Default constructor as an empty BayesNet */
    BayesNet() {};

  };

}
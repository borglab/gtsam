/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteBayesNet.h
 * @date Feb 15, 2011
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/discrete/DiscreteConditional.h>

namespace gtsam {

/** A Bayes net made from linear-Discrete densities */
  class GTSAM_EXPORT DiscreteBayesNet: public BayesNet<DiscreteConditional>
  {
  public:

    typedef FactorGraph<DiscreteConditional> Base;
    typedef DiscreteBayesNet This;
    typedef DiscreteConditional ConditionalType;
    typedef boost::shared_ptr<This> shared_ptr;
    typedef boost::shared_ptr<ConditionalType> sharedConditional;

    /// @name Standard Constructors
    /// @{

    /** Construct empty factor graph */
    DiscreteBayesNet() {}

    /** Construct from iterator over conditionals */
    template<typename ITERATOR>
    DiscreteBayesNet(ITERATOR firstConditional, ITERATOR lastConditional) : Base(firstConditional, lastConditional) {}

    /** Construct from container of factors (shared_ptr or plain objects) */
    template<class CONTAINER>
    explicit DiscreteBayesNet(const CONTAINER& conditionals) : Base(conditionals) {}

    /** Implicit copy/downcast constructor to override explicit template container constructor */
    template<class DERIVEDCONDITIONAL>
    DiscreteBayesNet(const FactorGraph<DERIVEDCONDITIONAL>& graph) : Base(graph) {}

    /// @}

    /// @name Testable
    /// @{

    /** Check equality */
    bool equals(const This& bn, double tol = 1e-9) const;

    /// @}

    /// @name Standard Interface
    /// @{

    /** Add a DiscreteCondtional */
    void add(const Signature& s);

//    /** Add a DiscreteCondtional in front, when listing parents first*/
//    GTSAM_EXPORT void add_front(const Signature& s);

    //** evaluate for given Values */
    double evaluate(const DiscreteConditional::Values & values) const;

    /**
    * Solve the DiscreteBayesNet by back-substitution
    */
    DiscreteFactor::sharedValues optimize() const;

    /** Do ancestral sampling */
    DiscreteFactor::sharedValues sample() const;

    ///@}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
  };

// traits
template<> struct traits<DiscreteBayesNet> : public Testable<DiscreteBayesNet> {};

} // \ namespace gtsam


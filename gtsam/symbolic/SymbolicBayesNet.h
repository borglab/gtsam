/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicBayesNet.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/base/types.h>

namespace gtsam {

  /** 
   * A SymbolicBayesNet is a Bayes Net of purely symbolic conditionals.
   * @ingroup symbolic
   */
  class SymbolicBayesNet : public BayesNet<SymbolicConditional> {
   public:
    typedef BayesNet<SymbolicConditional> Base;
    typedef SymbolicBayesNet This;
    typedef SymbolicConditional ConditionalType;
    typedef boost::shared_ptr<This> shared_ptr;
    typedef boost::shared_ptr<ConditionalType> sharedConditional;

    /// @name Standard Constructors
    /// @{

    /** Construct empty factor graph */
    SymbolicBayesNet() {}

    /** Construct from iterator over conditionals */
    template <typename ITERATOR>
    SymbolicBayesNet(ITERATOR firstConditional, ITERATOR lastConditional)
        : Base(firstConditional, lastConditional) {}

    /** Construct from container of factors (shared_ptr or plain objects) */
    template <class CONTAINER>
    explicit SymbolicBayesNet(const CONTAINER& conditionals) {
      push_back(conditionals);
    }

    /** Implicit copy/downcast constructor to override explicit template
     * container constructor */
    template <class DERIVEDCONDITIONAL>
    explicit SymbolicBayesNet(const FactorGraph<DERIVEDCONDITIONAL>& graph)
        : Base(graph) {}

    /// Destructor
    virtual ~SymbolicBayesNet() {}

    /// @}

    /// @name Testable
    /// @{

    /** Check equality */
    GTSAM_EXPORT bool equals(const This& bn, double tol = 1e-9) const;

    /// print
    GTSAM_EXPORT void print(
        const std::string& s = "SymbolicBayesNet",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override {
      Base::print(s, formatter);
    }

    /// @}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
};

  /// traits
  template<>
  struct traits<SymbolicBayesNet> : public Testable<SymbolicBayesNet> {
  };

} //\ namespace gtsam

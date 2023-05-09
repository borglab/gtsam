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
    typedef std::shared_ptr<This> shared_ptr;
    typedef std::shared_ptr<ConditionalType> sharedConditional;

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

    /**
     * Constructor that takes an initializer list of shared pointers.
     *  SymbolicBayesNet bn = {make_shared<SymbolicConditional>(), ...};
     */
    SymbolicBayesNet(std::initializer_list<std::shared_ptr<SymbolicConditional>> conditionals)
        : Base(conditionals) {}

    /// Construct from a single conditional
    SymbolicBayesNet(SymbolicConditional&& c) {
      emplace_shared<SymbolicConditional>(c);
    }

    /**
     * @brief Add a single conditional and return a reference.
     * This allows for chaining, e.g.,
     *   SymbolicBayesNet bn = 
     *     SymbolicBayesNet(SymbolicConditional(...))(SymbolicConditional(...));
     */
    SymbolicBayesNet& operator()(SymbolicConditional&& c) {
      emplace_shared<SymbolicConditional>(c);
      return *this;
    }

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
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
#endif
};

  /// traits
  template<>
  struct traits<SymbolicBayesNet> : public Testable<SymbolicBayesNet> {
  };

} //\ namespace gtsam

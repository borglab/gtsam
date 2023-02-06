/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicFactorGraph.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/base/types.h>

namespace gtsam {

  class SymbolicFactorGraph;
  class SymbolicConditional;
  class SymbolicBayesNet;
  class SymbolicEliminationTree;
  class SymbolicBayesTree;
  class SymbolicJunctionTree;

  /* ************************************************************************* */
  template<> struct EliminationTraits<SymbolicFactorGraph>
  {
    typedef SymbolicFactor FactorType;                   ///< Type of factors in factor graph
    typedef SymbolicFactorGraph FactorGraphType;         ///< Type of the factor graph (e.g. GaussianFactorGraph)
    typedef SymbolicConditional ConditionalType;         ///< Type of conditionals from elimination
    typedef SymbolicBayesNet BayesNetType;               ///< Type of Bayes net from sequential elimination
    typedef SymbolicEliminationTree EliminationTreeType; ///< Type of elimination tree
    typedef SymbolicBayesTree BayesTreeType;             ///< Type of Bayes tree
    typedef SymbolicJunctionTree JunctionTreeType;       ///< Type of Junction tree
    /// The default dense elimination function
    static std::pair<std::shared_ptr<ConditionalType>, std::shared_ptr<FactorType> >
      DefaultEliminate(const FactorGraphType& factors, const Ordering& keys) {
        return EliminateSymbolic(factors, keys); }
    /// The default ordering generation function
    static Ordering DefaultOrderingFunc(
        const FactorGraphType& graph,
        std::optional<std::reference_wrapper<const VariableIndex>> variableIndex) {
      return Ordering::Colamd((*variableIndex).get());
    }
  };

  /* ************************************************************************* */
  /** Symbolic Factor Graph
   *  \nosubgrouping
   */
  class GTSAM_EXPORT SymbolicFactorGraph :
    public FactorGraph<SymbolicFactor>,
    public EliminateableFactorGraph<SymbolicFactorGraph>
  {
  public:

    typedef SymbolicFactorGraph This; ///< Typedef to this class
    typedef FactorGraph<SymbolicFactor> Base; ///< Typedef to base factor graph type
    typedef EliminateableFactorGraph<This> BaseEliminateable; ///< Typedef to base elimination class
    typedef std::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

    /// @name Standard Constructors
    /// @{

    /** Construct empty factor graph */
    SymbolicFactorGraph() {}

    /** Construct from iterator over factors */
    template<typename ITERATOR>
    SymbolicFactorGraph(ITERATOR firstFactor, ITERATOR lastFactor) : Base(firstFactor, lastFactor) {}

    /** Construct from container of factors (shared_ptr or plain objects) */
    template<class CONTAINER>
    explicit SymbolicFactorGraph(const CONTAINER& factors) : Base(factors) {}

    /** Implicit copy/downcast constructor to override explicit template container constructor */
    template<class DERIVEDFACTOR>
    SymbolicFactorGraph(const FactorGraph<DERIVEDFACTOR>& graph) : Base(graph) {}

    /**
     * Constructor that takes an initializer list of shared pointers.
     *  FactorGraph fg = {make_shared<MyFactor>(), ...};
     */
    SymbolicFactorGraph(
        std::initializer_list<std::shared_ptr<SymbolicFactor>> sharedFactors)
        : Base(sharedFactors) {}

    /// Construct from a single factor
    SymbolicFactorGraph(SymbolicFactor&& c) {
        emplace_shared<SymbolicFactor>(c);
    }

    /**
     * @brief Add a single factor and return a reference.
     * This allows for chaining, e.g.,
     *   SymbolicFactorGraph bn =
     *     SymbolicFactorGraph(SymbolicFactor(...))(SymbolicFactor(...));
     */
    SymbolicFactorGraph& operator()(SymbolicFactor&& c) {
        emplace_shared<SymbolicFactor>(c);
        return *this;
    }

    /// Destructor
    virtual ~SymbolicFactorGraph() {}

    /// @}

    /// @name Testable
    /// @{

    bool equals(const This& fg, double tol = 1e-9) const;

    /// print
    void print(
        const std::string& s = "SymbolicFactorGraph",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override {
      Base::print(s, formatter);
    }

    /// @}

    /// @name Standard Interface
    /// @{

    /** Push back unary factor */
    void push_factor(Key key);

    /** Push back binary factor */
    void push_factor(Key key1, Key key2);

    /** Push back ternary factor */
    void push_factor(Key key1, Key key2, Key key3);

    /** Push back 4-way factor */
    void push_factor(Key key1, Key key2, Key key3, Key key4);

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
struct traits<SymbolicFactorGraph> : public Testable<SymbolicFactorGraph> {
};

} //\ namespace gtsam

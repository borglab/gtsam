/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DecisionTree.h
 * @brief   Decision Tree for use in DiscreteFactors
 * @author  Frank Dellaert
 * @author  Can Erdogan
 * @date    Jan 30, 2012
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/discrete/Assignment.h>

#include <boost/function.hpp>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <set>

namespace gtsam {

  /**
   * Decision Tree
   * L = label for variables
   * Y = function range (any algebra), e.g., bool, int, double
   */
  template<typename L, typename Y>
  class GTSAM_EXPORT DecisionTree {

   protected:
    /// Default method for comparison of two objects of type Y.
    static bool DefaultCompare(const Y& a, const Y& b) {
      return a == b;
    }

  public:

    using LabelFormatter = std::function<std::string(L)>;
    using ValueFormatter = std::function<std::string(Y)>;
    using CompareFunc = std::function<bool(const Y&, const Y&)>;

    /** Handy typedefs for unary and binary function types */
    using Unary = std::function<Y(const Y&)>;
    using Binary = std::function<Y(const Y&, const Y&)>;

    /** A label annotated with cardinality */
    using LabelC = std::pair<L,size_t>;
    using LabelCs = std::map<L, size_t>;

    /** DTs consist of Leaf and Choice nodes, both subclasses of Node */
    class Leaf;
    class Choice;

    /** ------------------------ Node base class --------------------------- */
    class Node {
    public:
      using Ptr = boost::shared_ptr<const Node>;

#ifdef DT_DEBUG_MEMORY
      static int nrNodes;
#endif

      // Constructor
      Node() {
#ifdef DT_DEBUG_MEMORY
      std::cout << ++nrNodes << " constructed " << id() << std::endl; std::cout.flush();
#endif
      }

      // Destructor
      virtual ~Node() {
#ifdef DT_DEBUG_MEMORY
      std::cout << --nrNodes << " destructed " << id() << std::endl; std::cout.flush();
#endif
      }

      // Unique ID for dot files
      const void* id() const { return this; }

      // everything else is virtual, no documentation here as internal
      virtual void print(const std::string& s,
                         const LabelFormatter& labelFormatter,
                         const ValueFormatter& valueFormatter) const = 0;
      virtual void dot(std::ostream& os, const LabelFormatter& labelFormatter,
                       const ValueFormatter& valueFormatter,
                       bool showZero) const = 0;
      virtual bool sameLeaf(const Leaf& q) const = 0;
      virtual bool sameLeaf(const Node& q) const = 0;
      virtual bool equals(const Node& other, const CompareFunc& compare =
                                                 &DefaultCompare) const = 0;
      virtual const Y& operator()(const Assignment<L>& x) const = 0;
      virtual Ptr apply(const Unary& op) const = 0;
      virtual Ptr apply_f_op_g(const Node&, const Binary&) const = 0;
      virtual Ptr apply_g_op_fL(const Leaf&, const Binary&) const = 0;
      virtual Ptr apply_g_op_fC(const Choice&, const Binary&) const = 0;
      virtual Ptr choose(const L& label, size_t index) const = 0;
      virtual bool isLeaf() const = 0;
    };
    /** ------------------------ Node base class --------------------------- */

  public:

    /** A function is a shared pointer to the root of a DT */
    using NodePtr = typename Node::Ptr;

    /// A DecisionTree just contains the root. TODO(dellaert): make protected.
    NodePtr root_;

  protected:

    /** Internal recursive function to create from keys, cardinalities, and Y values */
    template<typename It, typename ValueIt>
    NodePtr create(It begin, It end, ValueIt beginY, ValueIt endY) const;

    /**
     * @brief Convert from a DecisionTree<M, X> to DecisionTree<L, Y>.
     * 
     * @tparam M The previous label type.
     * @tparam X The previous value type.
     * @param f The node pointer to the root of the previous DecisionTree.
     * @param L_of_M Functor to convert from label type M to type L.
     * @param Y_of_X Functor to convert from value type X to type Y.
     * @return NodePtr 
     */
    template <typename M, typename X>
    NodePtr convertFrom(const typename DecisionTree<M, X>::NodePtr& f,
                        std::function<L(const M&)> L_of_M,
                        std::function<Y(const X&)> Y_of_X) const;

   public:

    /// @name Standard Constructors
    /// @{

    /** Default constructor (for serialization) */
    DecisionTree();

    /** Create a constant */
    DecisionTree(const Y& y);

    /** Create a new leaf function splitting on a variable */
    DecisionTree(const L& label, const Y& y1, const Y& y2);

    /** Allow Label+Cardinality for convenience */
    DecisionTree(const LabelC& label, const Y& y1, const Y& y2);

    /** Create from keys and a corresponding vector of values */
    DecisionTree(const LabelCs& labelCs, const std::vector<Y>& ys);

    /** Create from keys and string table */
    DecisionTree(const LabelCs& labelCs, const std::string& table);

    /** Create DecisionTree from others */
    template<typename Iterator>
    DecisionTree(Iterator begin, Iterator end, const L& label);

    /** Create DecisionTree from two others */
    DecisionTree(const L& label, //
        const DecisionTree& f0, const DecisionTree& f1);

    /**
     * @brief Convert from a different value type.
     *
     * @tparam X The previous value type.
     * @param other The DecisionTree to convert from.
     * @param Y_of_X Functor to convert from value type X to type Y.
     */
    template <typename X, typename Func>
    DecisionTree(const DecisionTree<L, X>& other, Func Y_of_X);

    /**
     * @brief Convert from a different value type X to value type Y, also transate
     * labels via map from type M to L.
     *
     * @tparam M Previous label type.
     * @tparam X Previous value type.
     * @param other The decision tree to convert.
     * @param L_of_M Map from label type M to type L.
     * @param Y_of_X Functor to convert from type X to type Y.
     */
    template <typename M, typename X, typename Func>
    DecisionTree(const DecisionTree<M, X>& other, const std::map<M, L>& map,
                 Func Y_of_X);

    /// @}
    /// @name Testable
    /// @{

    /**
     * @brief GTSAM-style print
     * 
     * @param s Prefix string.
     * @param labelFormatter Functor to format the node label.
     * @param valueFormatter Functor to format the node value.
     */
    void print(const std::string& s, const LabelFormatter& labelFormatter,
               const ValueFormatter& valueFormatter) const;

    // Testable
    bool equals(const DecisionTree& other,
                const CompareFunc& compare = &DefaultCompare) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /** Make virtual */
    virtual ~DecisionTree() {
    }

    /// Check if tree is empty.
    bool empty() const { return !root_; }

    /** equality */
    bool operator==(const DecisionTree& q) const;

    /** evaluate */
    const Y& operator()(const Assignment<L>& x) const;

    /**
     * @brief Visit all leaves in depth-first fashion.
     * 
     * @param f side-effect taking a value.
     * 
     * Example:
     *   int sum = 0;
     *   auto visitor = [&](int y) { sum += y; };
     *   tree.visitWith(visitor);
     */
    template <typename Func>
    void visit(Func f) const;

    /**
     * @brief Visit all leaves in depth-first fashion.
     * 
     * @param f side-effect taking an assignment and a value.
     * 
     * Example:
     *   int sum = 0;
     *   auto visitor = [&](const Assignment<L>& choices, int y) { sum += y; };
     *   tree.visitWith(visitor);
     */
    template <typename Func>
    void visitWith(Func f) const;

    /**
     * @brief Fold a binary function over the tree, returning accumulator.
     *
     * @tparam X type for accumulator.
     * @param f binary function: Y * X -> X returning an updated accumulator.
     * @param x0 initial value for accumulator.
     * @return X final value for accumulator.
     * 
     * @note X is always passed by value.
     * 
     * Example:
     *   auto add = [](const double& y, double x) { return y + x; };
     *   double sum = tree.fold(add, 0.0);
     */
    template <typename Func, typename X>
    X fold(Func f, X x0) const;

    /** Retrieve all unique labels as a set. */
    std::set<L> labels() const;

    /** apply Unary operation "op" to f */
    DecisionTree apply(const Unary& op) const;

    /** apply binary operation "op" to f and g */
    DecisionTree apply(const DecisionTree& g, const Binary& op) const;

    /** create a new function where value(label)==index
     * It's like "restrict" in Darwiche09book pg329, 330? */
    DecisionTree choose(const L& label, size_t index) const {
      NodePtr newRoot = root_->choose(label, index);
      return DecisionTree(newRoot);
    }

    /** combine subtrees on key with binary operation "op" */
    DecisionTree combine(const L& label, size_t cardinality, const Binary& op) const;

    /** combine with LabelC for convenience */
    DecisionTree combine(const LabelC& labelC, const Binary& op) const {
      return combine(labelC.first, labelC.second, op);
    }

    /** output to graphviz format, stream version */
    void dot(std::ostream& os, const LabelFormatter& labelFormatter,
             const ValueFormatter& valueFormatter, bool showZero = true) const;

    /** output to graphviz format, open a file */
    void dot(const std::string& name, const LabelFormatter& labelFormatter,
             const ValueFormatter& valueFormatter, bool showZero = true) const;

    /** output to graphviz format string */
    std::string dot(const LabelFormatter& labelFormatter,
                    const ValueFormatter& valueFormatter,
                    bool showZero = true) const;

    /// @name Advanced Interface
    /// @{

    // internal use only
    DecisionTree(const NodePtr& root);

    // internal use only
    template<typename Iterator> NodePtr
    compose(Iterator begin, Iterator end, const L& label) const;

    /// @}

  }; // DecisionTree

  /** free versions of apply */

  /// Apply unary operator `op` to DecisionTree `f`.
  template<typename L, typename Y>
  DecisionTree<L, Y> apply(const DecisionTree<L, Y>& f,
      const typename DecisionTree<L, Y>::Unary& op) {
    return f.apply(op);
  }

  /// Apply binary operator `op` to DecisionTree `f`.
  template<typename L, typename Y>
  DecisionTree<L, Y> apply(const DecisionTree<L, Y>& f,
      const DecisionTree<L, Y>& g,
      const typename DecisionTree<L, Y>::Binary& op) {
    return f.apply(g, op);
  }

} // namespace gtsam

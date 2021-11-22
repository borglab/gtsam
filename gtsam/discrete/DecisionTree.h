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

#include <gtsam/discrete/Assignment.h>

#include <boost/function.hpp>
#include <functional>
#include <iostream>
#include <map>
#include <vector>

namespace gtsam {

  /**
   * Decision Tree
   * L = label for variables
   * Y = function range (any algebra), e.g., bool, int, double
   */
  template<typename L, typename Y>
  class DecisionTree {

  public:

    /** Handy typedefs for unary and binary function types */
    typedef std::function<Y(const Y&)> Unary;
    typedef std::function<Y(const Y&, const Y&)> Binary;

    /** A label annotated with cardinality */
    typedef std::pair<L,size_t> LabelC;

    /** DTs consist of Leaf and Choice nodes, both subclasses of Node */
    class Leaf;
    class Choice;

    /** ------------------------ Node base class --------------------------- */
    class Node {
    public:
      typedef boost::shared_ptr<const Node> Ptr;

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
      virtual void print(const std::string& s = "") const = 0;
      virtual void dot(std::ostream& os, bool showZero) const = 0;
      virtual bool sameLeaf(const Leaf& q) const = 0;
      virtual bool sameLeaf(const Node& q) const = 0;
      virtual bool equals(const Node& other, double tol = 1e-9) const = 0;
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
    typedef typename Node::Ptr NodePtr;

    /* a DecisionTree just contains the root */
    NodePtr root_;

  protected:

    /** Internal recursive function to create from keys, cardinalities, and Y values */
    template<typename It, typename ValueIt>
    NodePtr create(It begin, It end, ValueIt beginY, ValueIt endY) const;

    /** Convert to a different type */
    template<typename M, typename X> NodePtr
    convert(const typename DecisionTree<M, X>::NodePtr& f, const std::map<M,
        L>& map, std::function<Y(const X&)> op);

    /** Default constructor */
    DecisionTree();

  public:

    /// @name Standard Constructors
    /// @{

    /** Create a constant */
    DecisionTree(const Y& y);

    /** Create a new leaf function splitting on a variable */
    DecisionTree(const L& label, const Y& y1, const Y& y2);

    /** Allow Label+Cardinality for convenience */
    DecisionTree(const LabelC& label, const Y& y1, const Y& y2);

    /** Create from keys and a corresponding vector of values */
    DecisionTree(const std::vector<LabelC>& labelCs, const std::vector<Y>& ys);

    /** Create from keys and string table */
    DecisionTree(const std::vector<LabelC>& labelCs, const std::string& table);

    /** Create DecisionTree from others */
    template<typename Iterator>
    DecisionTree(Iterator begin, Iterator end, const L& label);

    /** Create DecisionTree from two others */
    DecisionTree(const L& label, //
        const DecisionTree& f0, const DecisionTree& f1);

    /** Convert from a different type */
    template<typename M, typename X>
    DecisionTree(const DecisionTree<M, X>& other,
        const std::map<M, L>& map, std::function<Y(const X&)> op);

    /// @}
    /// @name Testable
    /// @{

    /** GTSAM-style print */
    void print(const std::string& s = "DecisionTree") const;

    // Testable
    bool equals(const DecisionTree& other, double tol = 1e-9) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /** Make virtual */
    virtual ~DecisionTree() {
    }

    /** equality */
    bool operator==(const DecisionTree& q) const;

    /** evaluate */
    const Y& operator()(const Assignment<L>& x) const;

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
    void dot(std::ostream& os, bool showZero = true) const;

    /** output to graphviz format, open a file */
    void dot(const std::string& name, bool showZero = true) const;

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

  template<typename Y, typename L>
  DecisionTree<L, Y> apply(const DecisionTree<L, Y>& f,
      const typename DecisionTree<L, Y>::Unary& op) {
    return f.apply(op);
  }

  template<typename Y, typename L>
  DecisionTree<L, Y> apply(const DecisionTree<L, Y>& f,
      const DecisionTree<L, Y>& g,
      const typename DecisionTree<L, Y>::Binary& op) {
    return f.apply(g, op);
  }

} // namespace gtsam

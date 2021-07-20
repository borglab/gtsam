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

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/base/Testable.h>

#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/vector.hpp>
using boost::assign::operator+=;
#include <boost/unordered_set.hpp>
#include <boost/noncopyable.hpp>

#include <list>
#include <cmath>
#include <fstream>
#include <sstream>

namespace gtsam {

  /*********************************************************************************/
  // Node
  /*********************************************************************************/
#ifdef DT_DEBUG_MEMORY
  template<typename L, typename Y>
  int DecisionTree<L, Y>::Node::nrNodes = 0;
#endif

  /*********************************************************************************/
  // Leaf
  /*********************************************************************************/
  template<typename L, typename Y>
  class DecisionTree<L, Y>::Leaf: public DecisionTree<L, Y>::Node {

    /** constant stored in this leaf */
    Y constant_;

  public:

    /** Constructor from constant */
    Leaf(const Y& constant) :
      constant_(constant) {}

    /** return the constant */
    const Y& constant() const {
      return constant_;
    }

    /// Leaf-Leaf equality
    bool sameLeaf(const Leaf& q) const override {
      return constant_ == q.constant_;
    }

    /// polymorphic equality: is q is a leaf, could be
    bool sameLeaf(const Node& q) const override {
      return (q.isLeaf() && q.sameLeaf(*this));
    }

    /** equality up to tolerance */
    bool equals(const Node& q, double tol) const override {
      const Leaf* other = dynamic_cast<const Leaf*> (&q);
      if (!other) return false;
      return std::abs(double(this->constant_ - other->constant_)) < tol;
    }

    /** print */
    void print(const std::string& s) const override {
      bool showZero = true;
      if (showZero || constant_) std::cout << s << " Leaf " << constant_ << std::endl;
    }

    /** to graphviz file */
    void dot(std::ostream& os, bool showZero) const override {
      if (showZero || constant_) os << "\"" << this->id() << "\" [label=\""
          << boost::format("%4.2g") % constant_
          << "\", shape=box, rank=sink, height=0.35, fixedsize=true]\n"; // width=0.55,
    }

    /** evaluate */
    const Y& operator()(const Assignment<L>& x) const override {
      return constant_;
    }

    /** apply unary operator */
    NodePtr apply(const Unary& op) const override {
      NodePtr f(new Leaf(op(constant_)));
      return f;
    }

    // Apply binary operator "h = f op g" on Leaf node
    // Note op is not assumed commutative so we need to keep track of order
    // Simply calls apply on argument to call correct virtual method:
    // fL.apply_f_op_g(gL) -> gL.apply_g_op_fL(fL) (below)
    // fL.apply_f_op_g(gC) -> gC.apply_g_op_fL(fL) (Choice)
    NodePtr apply_f_op_g(const Node& g, const Binary& op) const override {
      return g.apply_g_op_fL(*this, op);
    }

    // Applying binary operator to two leaves results in a leaf
    NodePtr apply_g_op_fL(const Leaf& fL, const Binary& op) const override {
      NodePtr h(new Leaf(op(fL.constant_, constant_))); // fL op gL
      return h;
    }

    // If second argument is a Choice node, call it's apply with leaf as second
    NodePtr apply_g_op_fC(const Choice& fC, const Binary& op) const override {
      return fC.apply_fC_op_gL(*this, op); // operand order back to normal
    }

    /** choose a branch, create new memory ! */
    NodePtr choose(const L& label, size_t index) const override {
      return NodePtr(new Leaf(constant()));
    }

    bool isLeaf() const override { return true; }

  }; // Leaf

  /*********************************************************************************/
  // Choice
  /*********************************************************************************/
  template<typename L, typename Y>
  class DecisionTree<L, Y>::Choice: public DecisionTree<L, Y>::Node {

    /** the label of the variable on which we split */
    L label_;

    /** The children of this Choice node. */
    std::vector<NodePtr> branches_;

  private:
    /** incremental allSame */
    size_t allSame_;

    typedef boost::shared_ptr<const Choice> ChoicePtr;

  public:

    ~Choice() override {
#ifdef DT_DEBUG_MEMORY
        std::std::cout << Node::nrNodes << " destructing (Choice) " << this->id() << std::std::endl;
#endif
    }

    /** If all branches of a choice node f are the same, just return a branch */
    static NodePtr Unique(const ChoicePtr& f) {
#ifndef DT_NO_PRUNING
      if (f->allSame_) {
        assert(f->branches().size() > 0);
        NodePtr f0 = f->branches_[0];
        assert(f0->isLeaf());
        NodePtr newLeaf(new Leaf(boost::dynamic_pointer_cast<const Leaf>(f0)->constant()));
        return newLeaf;
      } else
#endif
        return f;
    }

    bool isLeaf() const override { return false; }

    /** Constructor, given choice label and mandatory expected branch count */
    Choice(const L& label, size_t count) :
      label_(label), allSame_(true) {
      branches_.reserve(count);
    }

    /**
     * Construct from applying binary op to two Choice nodes
     */
    Choice(const Choice& f, const Choice& g, const Binary& op) :
      allSame_(true) {

      // Choose what to do based on label
      if (f.label() > g.label()) {
        // f higher than g
        label_ = f.label();
        size_t count = f.nrChoices();
        branches_.reserve(count);
        for (size_t i = 0; i < count; i++)
          push_back(f.branches_[i]->apply_f_op_g(g, op));
      } else if (g.label() > f.label()) {
        // f lower than g
        label_ = g.label();
        size_t count = g.nrChoices();
        branches_.reserve(count);
        for (size_t i = 0; i < count; i++)
          push_back(g.branches_[i]->apply_g_op_fC(f, op));
      } else {
        // f same level as g
        label_ = f.label();
        size_t count = f.nrChoices();
        branches_.reserve(count);
        for (size_t i = 0; i < count; i++)
          push_back(f.branches_[i]->apply_f_op_g(*g.branches_[i], op));
      }
    }

    const L& label() const {
      return label_;
    }

    size_t nrChoices() const {
      return branches_.size();
    }

    const std::vector<NodePtr>& branches() const {
      return branches_;
    }

    /** add a branch: TODO merge into constructor */
    void push_back(const NodePtr& node) {
      // allSame_ is restricted to leaf nodes in a decision tree
      if (allSame_ && !branches_.empty()) {
        allSame_ = node->sameLeaf(*branches_.back());
      }
      branches_.push_back(node);
    }

    /** print (as a tree) */
    void print(const std::string& s) const override {
      std::cout << s << " Choice(";
      //        std::cout << this << ",";
      std::cout << label_ << ") " << std::endl;
      for (size_t i = 0; i < branches_.size(); i++)
        branches_[i]->print((boost::format("%s %d") % s % i).str());
    }

    /** output to graphviz (as a a graph) */
    void dot(std::ostream& os, bool showZero) const override {
      os << "\"" << this->id() << "\" [shape=circle, label=\"" << label_
          << "\"]\n";
      for (size_t i = 0; i < branches_.size(); i++) {
        NodePtr branch = branches_[i];

        // Check if zero
        if (!showZero) {
          const Leaf* leaf = dynamic_cast<const Leaf*> (branch.get());
          if (leaf && !leaf->constant()) continue;
        }

        os << "\"" << this->id() << "\" -> \"" << branch->id() << "\"";
        if (i == 0) os << " [style=dashed]";
        if (i > 1) os << " [style=bold]";
        os << std::endl;
        branch->dot(os, showZero);
      }
    }

    /// Choice-Leaf equality: always false
    bool sameLeaf(const Leaf& q) const override {
      return false;
    }

    /// polymorphic equality: if q is a leaf, could be...
    bool sameLeaf(const Node& q) const override {
      return (q.isLeaf() && q.sameLeaf(*this));
    }

    /** equality up to tolerance */
    bool equals(const Node& q, double tol) const override {
      const Choice* other = dynamic_cast<const Choice*> (&q);
      if (!other) return false;
      if (this->label_ != other->label_) return false;
      if (branches_.size() != other->branches_.size()) return false;
      // we don't care about shared pointers being equal here
      for (size_t i = 0; i < branches_.size(); i++)
        if (!(branches_[i]->equals(*(other->branches_[i]), tol))) return false;
      return true;
    }

    /** evaluate */
    const Y& operator()(const Assignment<L>& x) const override {
#ifndef NDEBUG
      typename Assignment<L>::const_iterator it = x.find(label_);
      if (it == x.end()) {
        std::cout << "Trying to find value for " << label_ << std::endl;
        throw std::invalid_argument(
            "DecisionTree::operator(): value undefined for a label");
      }
#endif
      size_t index = x.at(label_);
      NodePtr child = branches_[index];
      return (*child)(x);
    }

    /**
     * Construct from applying unary op to a Choice node
     */
    Choice(const L& label, const Choice& f, const Unary& op) :
      label_(label), allSame_(true) {

      branches_.reserve(f.branches_.size()); // reserve space
      for (const NodePtr& branch: f.branches_)
              push_back(branch->apply(op));
    }

    /** apply unary operator */
    NodePtr apply(const Unary& op) const override {
      boost::shared_ptr<Choice> r(new Choice(label_, *this, op));
      return Unique(r);
    }

    // Apply binary operator "h = f op g" on Choice node
    // Note op is not assumed commutative so we need to keep track of order
    // Simply calls apply on argument to call correct virtual method:
    // fC.apply_f_op_g(gL) -> gL.apply_g_op_fC(fC) -> (Leaf)
    // fC.apply_f_op_g(gC) -> gC.apply_g_op_fC(fC) -> (below)
    NodePtr apply_f_op_g(const Node& g, const Binary& op) const override {
      return g.apply_g_op_fC(*this, op);
    }

    // If second argument of binary op is Leaf node, recurse on branches
    NodePtr apply_g_op_fL(const Leaf& fL, const Binary& op) const override {
      boost::shared_ptr<Choice> h(new Choice(label(), nrChoices()));
      for(NodePtr branch: branches_)
              h->push_back(fL.apply_f_op_g(*branch, op));
      return Unique(h);
    }

    // If second argument of binary op is Choice, call constructor
    NodePtr apply_g_op_fC(const Choice& fC, const Binary& op) const override {
      boost::shared_ptr<Choice> h(new Choice(fC, *this, op));
      return Unique(h);
    }

    // If second argument of binary op is Leaf
    template<typename OP>
    NodePtr apply_fC_op_gL(const Leaf& gL, OP op) const {
      boost::shared_ptr<Choice> h(new Choice(label(), nrChoices()));
      for(const NodePtr& branch: branches_)
              h->push_back(branch->apply_f_op_g(gL, op));
      return Unique(h);
    }

    /** choose a branch, recursively */
    NodePtr choose(const L& label, size_t index) const override {
      if (label_ == label)
        return branches_[index]; // choose branch

      // second case, not label of interest, just recurse
      boost::shared_ptr<Choice> r(new Choice(label_, branches_.size()));
      for(const NodePtr& branch: branches_)
              r->push_back(branch->choose(label, index));
      return Unique(r);
    }

  }; // Choice

  /*********************************************************************************/
  // DecisionTree
  /*********************************************************************************/
  template<typename L, typename Y>
  DecisionTree<L, Y>::DecisionTree() {
  }

  template<typename L, typename Y>
  DecisionTree<L, Y>::DecisionTree(const NodePtr& root) :
    root_(root) {
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  DecisionTree<L, Y>::DecisionTree(const Y& y)  {
    root_ = NodePtr(new Leaf(y));
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  DecisionTree<L, Y>::DecisionTree(//
      const L& label, const Y& y1, const Y& y2)  {
    boost::shared_ptr<Choice> a(new Choice(label, 2));
    NodePtr l1(new Leaf(y1)), l2(new Leaf(y2));
    a->push_back(l1);
    a->push_back(l2);
    root_ = Choice::Unique(a);
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  DecisionTree<L, Y>::DecisionTree(//
      const LabelC& labelC, const Y& y1, const Y& y2)  {
    if (labelC.second != 2) throw std::invalid_argument(
        "DecisionTree: binary constructor called with non-binary label");
    boost::shared_ptr<Choice> a(new Choice(labelC.first, 2));
    NodePtr l1(new Leaf(y1)), l2(new Leaf(y2));
    a->push_back(l1);
    a->push_back(l2);
    root_ = Choice::Unique(a);
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  DecisionTree<L, Y>::DecisionTree(const std::vector<LabelC>& labelCs,
      const std::vector<Y>& ys) {
    // call recursive Create
    root_ = create(labelCs.begin(), labelCs.end(), ys.begin(), ys.end());
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  DecisionTree<L, Y>::DecisionTree(const std::vector<LabelC>& labelCs,
      const std::string& table) {

    // Convert std::string to values of type Y
    std::vector<Y> ys;
    std::istringstream iss(table);
    copy(std::istream_iterator<Y>(iss), std::istream_iterator<Y>(),
        back_inserter(ys));

    // now call recursive Create
    root_ = create(labelCs.begin(), labelCs.end(), ys.begin(), ys.end());
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  template<typename Iterator> DecisionTree<L, Y>::DecisionTree(
      Iterator begin, Iterator end, const L& label) {
    root_ = compose(begin, end, label);
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  DecisionTree<L, Y>::DecisionTree(const L& label,
      const DecisionTree& f0, const DecisionTree& f1)  {
    std::vector<DecisionTree> functions;
    functions += f0, f1;
    root_ = compose(functions.begin(), functions.end(), label);
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  template<typename M, typename X>
  DecisionTree<L, Y>::DecisionTree(const DecisionTree<M, X>& other,
      const std::map<M, L>& map, std::function<Y(const X&)> op)  {
    root_ = convert(other.root_, map, op);
  }

  /*********************************************************************************/
  // Called by two constructors above.
  // Takes a label and a corresponding range of decision trees, and creates a new
  // decision tree. However, the order of the labels needs to be respected, so we
  // cannot just create a root Choice node on the label: if the label is not the
  // highest label, we need to do a complicated and expensive recursive call.
  template<typename L, typename Y> template<typename Iterator>
  typename DecisionTree<L, Y>::NodePtr DecisionTree<L, Y>::compose(Iterator begin,
      Iterator end, const L& label) const {

    // find highest label among branches
    boost::optional<L> highestLabel;
    size_t nrChoices = 0;
    for (Iterator it = begin; it != end; it++) {
      if (it->root_->isLeaf())
        continue;
      boost::shared_ptr<const Choice> c =
          boost::dynamic_pointer_cast<const Choice>(it->root_);
      if (!highestLabel || c->label() > *highestLabel) {
        highestLabel.reset(c->label());
        nrChoices = c->nrChoices();
      }
    }

    // if label is already in correct order, just put together a choice on label
    if (!nrChoices || !highestLabel || label > *highestLabel) {
      boost::shared_ptr<Choice> choiceOnLabel(new Choice(label, end - begin));
      for (Iterator it = begin; it != end; it++)
        choiceOnLabel->push_back(it->root_);
      return Choice::Unique(choiceOnLabel);
    } else {
      // Set up a new choice on the highest label
      boost::shared_ptr<Choice> choiceOnHighestLabel(new Choice(*highestLabel, nrChoices));
      // now, for all possible values of highestLabel
      for (size_t index = 0; index < nrChoices; index++) {
        // make a new set of functions for composing by iterating over the given
        // functions, and selecting the appropriate branch.
        std::vector<DecisionTree> functions;
        for (Iterator it = begin; it != end; it++) {
          // by restricting the input functions to value i for labelBelow
          DecisionTree chosen = it->choose(*highestLabel, index);
          functions.push_back(chosen);
        }
        // We then recurse, for all values of the highest label
        NodePtr fi = compose(functions.begin(), functions.end(), label);
        choiceOnHighestLabel->push_back(fi);
      }
      return Choice::Unique(choiceOnHighestLabel);
    }
  }

  /*********************************************************************************/
  // "create" is a bit of a complicated thing, but very useful.
  // It takes a range of labels and a corresponding range of values,
  // and creates a decision tree, as follows:
  // - if there is only one label, creates a choice node with values in leaves
  // - otherwise, it evenly splits up the range of values and creates a tree for
  //   each sub-range, and assigns that tree to first label's choices
  // Example:
  // create([B A],[1 2 3 4]) would call
  //   create([A],[1 2])
  //   create([A],[3 4])
  // and produce
  // B=0
  //  A=0: 1
  //  A=1: 2
  // B=1
  //  A=0: 3
  //  A=1: 4
  // Note, through the magic of "compose", create([A B],[1 2 3 4]) will produce
  // exactly the same tree as above: the highest label is always the root.
  // However, it will be *way* faster if labels are given highest to lowest.
  template<typename L, typename Y>
  template<typename It, typename ValueIt>
  typename DecisionTree<L, Y>::NodePtr DecisionTree<L, Y>::create(
      It begin, It end, ValueIt beginY, ValueIt endY) const {

    // get crucial counts
    size_t nrChoices = begin->second;
    size_t size = endY - beginY;

    // Find the next key to work on
    It labelC = begin + 1;
    if (labelC == end) {
      // Base case: only one key left
      // Create a simple choice node with values as leaves.
      if (size != nrChoices) {
        std::cout << "Trying to create DD on " << begin->first << std::endl;
        std::cout << boost::format("DecisionTree::create: expected %d values but got %d instead") % nrChoices % size << std::endl;
        throw std::invalid_argument("DecisionTree::create invalid argument");
      }
      boost::shared_ptr<Choice> choice(new Choice(begin->first, endY - beginY));
      for (ValueIt y = beginY; y != endY; y++)
        choice->push_back(NodePtr(new Leaf(*y)));
      return Choice::Unique(choice);
    }

    // Recursive case: perform "Shannon expansion"
    // Creates one tree (i.e.,function) for each choice of current key
    // by calling create recursively, and then puts them all together.
    std::vector<DecisionTree> functions;
    size_t split = size / nrChoices;
    for (size_t i = 0; i < nrChoices; i++, beginY += split) {
      NodePtr f = create<It, ValueIt>(labelC, end, beginY, beginY + split);
      functions += DecisionTree(f);
    }
    return compose(functions.begin(), functions.end(), begin->first);
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  template<typename M, typename X>
  typename DecisionTree<L, Y>::NodePtr DecisionTree<L, Y>::convert(
      const typename DecisionTree<M, X>::NodePtr& f, const std::map<M, L>& map,
      std::function<Y(const X&)> op) {

    typedef DecisionTree<M, X> MX;
    typedef typename MX::Leaf MXLeaf;
    typedef typename MX::Choice MXChoice;
    typedef typename MX::NodePtr MXNodePtr;
    typedef DecisionTree<L, Y> LY;

    // ugliness below because apparently we can't have templated virtual functions
    // If leaf, apply unary conversion "op" and create a unique leaf
    const MXLeaf* leaf = dynamic_cast<const MXLeaf*> (f.get());
    if (leaf) return NodePtr(new Leaf(op(leaf->constant())));

    // Check if Choice
    boost::shared_ptr<const MXChoice> choice = boost::dynamic_pointer_cast<const MXChoice> (f);
    if (!choice) throw std::invalid_argument(
        "DecisionTree::Convert: Invalid NodePtr");

    // get new label
    M oldLabel = choice->label();
    L newLabel = map.at(oldLabel);

    // put together via Shannon expansion otherwise not sorted.
    std::vector<LY> functions;
    for(const MXNodePtr& branch: choice->branches()) {
      LY converted(convert<M, X>(branch, map, op));
      functions += converted;
    }
    return LY::compose(functions.begin(), functions.end(), newLabel);
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  bool DecisionTree<L, Y>::equals(const DecisionTree& other, double tol) const {
    return root_->equals(*other.root_, tol);
  }

  template<typename L, typename Y>
  void DecisionTree<L, Y>::print(const std::string& s) const {
    root_->print(s);
  }

  template<typename L, typename Y>
  bool DecisionTree<L, Y>::operator==(const DecisionTree& other) const {
    return root_->equals(*other.root_);
  }

  template<typename L, typename Y>
  const Y& DecisionTree<L, Y>::operator()(const Assignment<L>& x) const {
    return root_->operator ()(x);
  }

  template<typename L, typename Y>
  DecisionTree<L, Y> DecisionTree<L, Y>::apply(const Unary& op) const {
    return DecisionTree(root_->apply(op));
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  DecisionTree<L, Y> DecisionTree<L, Y>::apply(const DecisionTree& g,
      const Binary& op) const {
    // apply the operaton on the root of both diagrams
    NodePtr h = root_->apply_f_op_g(*g.root_, op);
    // create a new class with the resulting root "h"
    DecisionTree result(h);
    return result;
  }

  /*********************************************************************************/
  // The way this works:
  // We have an ADT, picture it as a tree.
  // At a certain depth, we have a branch on "label".
  // The function "choose(label,index)" will return a tree of one less depth,
  // where there is no more branch on "label": only the subtree under that
  // branch point corresponding to the value "index" is left instead.
  // The function below get all these smaller trees and "ops" them together.
  // This implements marginalization in Darwiche09book, pg 330
  template<typename L, typename Y>
  DecisionTree<L, Y> DecisionTree<L, Y>::combine(const L& label,
      size_t cardinality, const Binary& op) const {
    DecisionTree result = choose(label, 0);
    for (size_t index = 1; index < cardinality; index++) {
      DecisionTree chosen = choose(label, index);
      result = result.apply(chosen, op);
    }
    return result;
  }

  /*********************************************************************************/
  template<typename L, typename Y>
  void DecisionTree<L, Y>::dot(std::ostream& os, bool showZero) const {
    os << "digraph G {\n";
    root_->dot(os, showZero);
    os << " [ordering=out]}" << std::endl;
  }

  template<typename L, typename Y>
  void DecisionTree<L, Y>::dot(const std::string& name, bool showZero) const {
    std::ofstream os((name + ".dot").c_str());
    dot(os, showZero);
    int result = system(
        ("dot -Tpdf " + name + ".dot -o " + name + ".pdf >& /dev/null").c_str());
    if (result==-1) throw std::runtime_error("DecisionTree::dot system call failed");
}

/*********************************************************************************/

} // namespace gtsam



/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BTree.h
 * @brief purely functional binary tree
 * @author Chris Beall
 * @author Frank Dellaert
 * @date Feb 3, 2010
 */

#include <stack>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace gtsam {

  /**
   * @brief Binary tree
   * @addtogroup base
   */
  template<class KEY, class VALUE>
  class BTree {

  public:

    typedef std::pair<KEY, VALUE> value_type;

  private:

    /**
     * Node in a tree
     */
    struct Node {

      const size_t height_;
      const value_type keyValue_;
      const BTree left, right;

      /** default constructor */
      Node() {
      }

      /**
       * Create leaf node with height 1
       * @param keyValue (key,value) pair
       */
      Node(const value_type& keyValue) :
        height_(1), keyValue_(keyValue) {
      }

      /**
       * Create a node from two subtrees and a key value pair
       */
      Node(const BTree& l, const value_type& keyValue, const BTree& r) :
        height_(l.height() >= r.height() ? l.height() + 1 : r.height() + 1),
        keyValue_(keyValue), left(l), right(r) {
      }

      inline const KEY& key() const { return keyValue_.first;}
      inline const VALUE& value() const { return keyValue_.second;}

    }; // Node

    // We store a shared pointer to the root of the functional tree
    // composed of Node classes. If root_==nullptr, the tree is empty.
    typedef boost::shared_ptr<const Node> sharedNode;
    sharedNode root_;

    inline const value_type& keyValue() const { return root_->keyValue_;}
    inline const KEY&        key()      const { return root_->key();    }
    inline const VALUE&      value()    const { return root_->value();  }
    inline const BTree&      left()     const { return root_->left;     }
    inline const BTree&      right()    const { return root_->right;    }

    /** create a new balanced tree out of two trees and a key-value pair */
    static BTree balance(const BTree& l, const value_type& xd, const BTree& r) {
      size_t hl = l.height(), hr = r.height();
      if (hl > hr + 2) {
        const BTree& ll = l.left(), lr = l.right();
        if (ll.height() >= lr.height())
          return BTree(ll, l.keyValue(), BTree(lr, xd, r));
        else {
          BTree _left(ll, l.keyValue(), lr.left());
          BTree _right(lr.right(), xd, r);
          return BTree(_left, lr.keyValue(), _right);
        }
      } else if (hr > hl + 2) {
        const BTree& rl = r.left(), rr = r.right();
        if (rr.height() >= rl.height())
          return BTree(BTree(l, xd, rl), r.keyValue(), rr);
        else {
          BTree _left(l, xd, rl.left());
          BTree _right(rl.right(), r.keyValue(), rr);
          return BTree(_left, rl.keyValue(), _right);
        }
      } else
        return BTree(l, xd, r);
    }

  public:

    /** default constructor creates an empty tree */
    BTree() {
    }

    /** copy constructor */
    BTree(const BTree& other) :
      root_(other.root_) {
    }

    /** create leaf from key-value pair */
    BTree(const value_type& keyValue) :
      root_(new Node(keyValue)) {
    }

    /** create from key-value pair and left, right subtrees */
    BTree(const BTree& l, const value_type& keyValue, const BTree& r) :
      root_(new Node(l, keyValue, r)) {
    }

    /** assignment operator */
    BTree & operator= (const BTree & other) {
      root_ = other.root_;
      return *this;
    }

    /** Check whether tree is empty */
    bool empty() const {
      return !root_;
    }

    /** add a key-value pair */
    BTree add(const value_type& xd) const {
      if (empty()) return BTree(xd);
      const KEY& x = xd.first;
      if (x == key())
        return BTree(left(), xd, right());
      else if (x < key())
        return balance(left().add(xd), keyValue(), right());
      else
        return balance(left(), keyValue(), right().add(xd));
    }

    /** add a key-value pair */
    BTree add(const KEY& x, const VALUE& d) const {
      return add(std::make_pair(x, d));
    }

    /** member predicate */
    bool mem(const KEY& x) const {
      if (!root_) return false;
      if (x == key()) return true;
      if (x < key())
        return left().mem(x);
      else
        return right().mem(x);
    }

    /** Check whether trees are *exactly* the same (occupy same memory) */
    inline bool same(const BTree& other) const {
      return (other.root_ == root_);
    }

    /**
     * Check whether trees are structurally the same,
     * i.e., contain the same values in same tree-structure.
     */
    bool operator==(const BTree& other) const {
      if (other.root_ == root_) return true; // if same, we're done
      if (empty() && !other.empty()) return false;
      if (!empty() && other.empty()) return false;
      // both non-empty, recurse: check this key-value pair and subtrees...
      return (keyValue() == other.keyValue()) && (left() == other.left())
          && (right() == other.right());
    }

    inline bool operator!=(const BTree& other) const {
      return !operator==(other);
    }

    /** minimum key binding */
    const value_type& min() const {
      if (!root_) throw std::invalid_argument("BTree::min: empty tree");
      if (left().empty()) return keyValue();
      return left().min();
    }

    /** remove minimum key binding */
    BTree remove_min() const {
      if (!root_) throw std::invalid_argument("BTree::remove_min: empty tree");
      if (left().empty()) return right();
      return balance(left().remove_min(), keyValue(), right());
    }

    /** merge two trees */
    static BTree merge(const BTree& t1, const BTree& t2) {
      if (t1.empty()) return t2;
      if (t2.empty()) return t1;
      const value_type& xd = t2.min();
      return balance(t1, xd, t2.remove_min());
    }

    /** remove a key-value pair */
    BTree remove(const KEY& x) const {
      if (!root_) return BTree();
      if (x == key())
        return merge(left(), right());
      else if (x < key())
        return balance(left().remove(x), keyValue(), right());
      else
        return balance(left(), keyValue(), right().remove(x));
    }

    /** Return height of the tree, 0 if empty */
    size_t height() const {
      return (root_ != nullptr) ? root_->height_ : 0;
    }

    /** return size of the tree */
    size_t size() const {
      if (!root_) return 0;
      return left().size() + 1 + right().size();
    }

    /**
     *  find a value given a key, throws exception when not found
     *  Optimized non-recursive version as [find] is crucial for speed
     */
    const VALUE& find(const KEY& k) const {
      const Node* node = root_.get();
      while (node) {
        const KEY& key = node->key();
        if      (k < key) node = node->left.root_.get();
        else if (key < k) node = node->right.root_.get();
        else return node->value();
      }

      throw std::invalid_argument("BTree::find: key not found");
    }

    /** print in-order */
    void print(const std::string& s = "") const {
      if (empty()) return;
      KEY k = key();
      std::stringstream ss;
      ss << height();
      k.print(s + ss.str() + " ");
      left().print(s + "L ");
      right().print(s + "R ");
    }

    /** iterate over tree */
    void iter(boost::function<void(const KEY&, const VALUE&)> f) const {
      if (!root_) return;
      left().iter(f);
      f(key(), value());
      right().iter(f);
    }

    /** map key-values in tree over function f that computes a new value */
    template<class TO>
    BTree<KEY, TO> map(boost::function<TO(const KEY&, const VALUE&)> f) const {
      if (empty()) return BTree<KEY, TO> ();
      std::pair<KEY, TO> xd(key(), f(key(), value()));
      return BTree<KEY, TO> (left().map(f), xd, right().map(f));
    }

    /**
     * t.fold(f,a) computes [(f kN dN ... (f k1 d1 a)...)],
     * where [k1 ... kN] are the keys of all bindings in [m],
     * and [d1 ... dN] are the associated data.
     * The associated values are passed to [f] in reverse sort order
     */
    template<class ACC>
    ACC fold(boost::function<ACC(const KEY&, const VALUE&, const ACC&)> f,
        const ACC& a) const {
      if (!root_) return a;
      ACC ar = right().fold(f, a); // fold over right subtree
      ACC am = f(key(), value(), ar); // apply f with current value
      return left().fold(f, am); // fold over left subtree
    }

    /**
     *  @brief Const iterator
     *  Not trivial: iterator keeps a stack to indicate current path from root_
     */
    class const_iterator {

    private:

      typedef const_iterator Self;
      typedef std::pair<sharedNode, bool> flagged;

      /** path to the iterator, annotated with flag */
      std::stack<flagged> path_;

      const sharedNode& current() const {
        return path_.top().first;
      }

      bool done() const {
        return path_.top().second;
      }

      // The idea is we already iterated through the left-subtree and current key-value.
      // We now try pushing left subtree of right onto the stack. If there is no right
      // sub-tree, we pop this node of the stack and the parent becomes the iterator.
      // We avoid going down a right-subtree that was already visited by checking the flag.
      void increment() {
        if (path_.empty()) return;
        sharedNode t = current()->right.root_;
        if (!t || done()) {
          // no right subtree, iterator becomes first parent with a non-visited right subtree
          path_.pop();
          while (!path_.empty() && done())
            path_.pop();
        } else {
          path_.top().second = true; // flag we visited right
          // push right root and its left-most path onto the stack
          while (t) {
            path_.push(std::make_pair(t, false));
            t = t->left.root_;
          }
        }
      }

    public:

      // traits for playing nice with STL
      typedef ptrdiff_t difference_type;
      typedef std::forward_iterator_tag iterator_category;
      typedef std::pair<KEY, VALUE> value_type;
      typedef const value_type* pointer;
      typedef const value_type& reference;

      /** initialize end */
      const_iterator() {
      }

      /** initialize from root */
      const_iterator(const sharedNode& root) {
        sharedNode t = root;
        while (t) {
          path_.push(std::make_pair(t, false));
          t = t->left.root_;
        }
      }

      /** equality */
      bool operator==(const Self& __x) const {
        return path_ == __x.path_;
      }

      /** inequality */
      bool operator!=(const Self& __x) const {
        return path_ != __x.path_;
      }

      /** dereference */
      reference operator*() const {
        if (path_.empty()) throw std::invalid_argument(
            "operator*: tried to dereference end");
        return current()->keyValue_;
      }

      /** dereference */
      pointer operator->() const {
        if (path_.empty()) throw std::invalid_argument(
            "operator->: tried to dereference end");
        return &(current()->keyValue_);
      }

      /** pre-increment */
      Self& operator++() {
        increment();
        return *this;
      }

      /** post-increment */
      Self operator++(int) {
        Self __tmp = *this;
        increment();
        return __tmp;
      }

    }; // const_iterator

    // to make BTree work with range-based for
    // We do *not* want a non-const iterator
    typedef const_iterator iterator;

    /** return iterator */
    const_iterator begin() const {
      return const_iterator(root_);
    }

    /** return iterator */
    const_iterator end() const {
      return const_iterator();
    }

  }; // BTree

} // namespace gtsam


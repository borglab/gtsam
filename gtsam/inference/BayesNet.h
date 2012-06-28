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
 */

// \callgraph

#pragma once

#include <list>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/assign/list_inserter.hpp>
#include <boost/lexical_cast.hpp>

#include <gtsam/base/types.h>
#include <gtsam/base/FastList.h>
#include <gtsam/inference/Permutation.h>

namespace gtsam {

/**
 * A BayesNet is a list of conditionals, stored in elimination order, i.e.
 * leaves first, parents last.  GaussianBayesNet and SymbolicBayesNet are
 * defined as typedefs of this class, using GaussianConditional and
 * IndexConditional as the CONDITIONAL template argument.
 *
 * todo:  Symbolic using Index is a misnomer.
 * todo:  how to handle Bayes nets with an optimize function?  Currently using global functions.
 * \nosubgrouping
 */
template<class CONDITIONAL>
class BayesNet {

public:

  typedef typename boost::shared_ptr<BayesNet<CONDITIONAL> > shared_ptr;

  /** We store shared pointers to Conditional densities */
  typedef typename CONDITIONAL::KeyType KeyType;
  typedef typename boost::shared_ptr<CONDITIONAL> sharedConditional;
  typedef typename boost::shared_ptr<const CONDITIONAL> const_sharedConditional;
  typedef typename std::list<sharedConditional> Conditionals;

  typedef typename Conditionals::iterator iterator;
  typedef typename Conditionals::reverse_iterator reverse_iterator;
  typedef typename Conditionals::const_iterator const_iterator;
  typedef typename Conditionals::const_reverse_iterator const_reverse_iterator;

protected:

  /**
   *  Conditional densities are stored in reverse topological sort order (i.e., leaves first,
   *  parents last), which corresponds to the elimination ordering if so obtained,
   *  and is consistent with the column (block) ordering of an upper triangular matrix.
   */
  Conditionals conditionals_;

public:

	/// @name Standard Constructors
	/// @{

  /** Default constructor as an empty BayesNet */
  BayesNet() {};

  /** convert from a derived type */
  template<class DERIVEDCONDITIONAL>
  BayesNet(const BayesNet<DERIVEDCONDITIONAL>& bn) {
    conditionals_.assign(bn.begin(), bn.end());
  }

  /** BayesNet with 1 conditional */
  explicit BayesNet(const sharedConditional& conditional) { push_back(conditional); }

	/// @}
	/// @name Testable
	/// @{

  /** print */
  void print(const std::string& s = "",
  		const IndexFormatter& formatter = DefaultIndexFormatter) const;

  /** print statistics */
  void printStats(const std::string& s = "") const;

  /** check equality */
  bool equals(const BayesNet& other, double tol = 1e-9) const;

	/// @}
	/// @name Standard Interface
	/// @{

  /** size is the number of nodes */
  size_t size() const {
    return conditionals_.size();
  }

  /** return keys in reverse topological sort order, i.e., elimination order */
  FastList<Index> ordering() const;

  /** SLOW O(n) random access to Conditional by key */
  sharedConditional operator[](Index key) const;

  /** return last node in ordering */
  boost::shared_ptr<const CONDITIONAL> front() const { return conditionals_.front(); }

  /** return last node in ordering */
  boost::shared_ptr<const CONDITIONAL> back() const { return conditionals_.back(); }

  /** return iterators. FD: breaks encapsulation? */
  const_iterator begin()          const {return conditionals_.begin();}   ///<TODO: comment
  const_iterator end()            const {return conditionals_.end();}		  ///<TODO: comment
  const_reverse_iterator rbegin() const {return conditionals_.rbegin();}  ///<TODO: comment
  const_reverse_iterator rend()   const {return conditionals_.rend();}  	///<TODO: comment

  /** Find an iterator pointing to the conditional where the specified key
   * appears as a frontal variable, or end() if no conditional contains this
   * key.  Running time is approximately \f$ O(n) \f$ in the number of
   * conditionals in the BayesNet.
   * @param key The index to find in the frontal variables of a conditional.
   */
  const_iterator find(Index key) const;

	/// @}
	/// @name Advanced Interface
	/// @{

  /**
   * Remove any leaf conditional.  The conditional to remove is specified by
   * iterator.  To find the iterator pointing to the conditional containing a
   * particular key, use find(), which has \f$ O(n) \f$ complexity.  The
   * popLeaf function by itself has \f$ O(1) \f$ complexity.
   *
   * If gtsam is compiled without NDEBUG defined, this function will check that
   * the node is indeed a leaf, but otherwise will not check, because the check
   * has \f$ O(n^2) \f$ complexity.
   *
   * Example 1:
     \code
     // Remove a leaf node with a known conditional
     GaussianBayesNet gbn = ...
     GaussianBayesNet::iterator leafConditional = ...
     gbn.popLeaf(leafConditional);
     \endcode
   * Example 2:
     \code
     // Remove the leaf node containing variable index 14
     GaussianBayesNet gbn = ...
     gbn.popLeaf(gbn.find(14));
     \endcode
   * @param conditional The iterator pointing to the leaf conditional to remove
   */
  void popLeaf(iterator conditional);

  /** return last node in ordering */
  sharedConditional& front() { return conditionals_.front(); }

  /** return last node in ordering */
  sharedConditional& back() { return conditionals_.back(); }

  /** Find an iterator pointing to the conditional where the specified key
   * appears as a frontal variable, or end() if no conditional contains this
   * key.  Running time is approximately \f$ O(n) \f$ in the number of
   * conditionals in the BayesNet.
   * @param key The index to find in the frontal variables of a conditional.
   */
  iterator find(Index key);

  /** push_back: use reverse topological sort (i.e. parents last / elimination order) */
  inline void push_back(const sharedConditional& conditional) {
    conditionals_.push_back(conditional);
  }

  /** push_front: use topological sort (i.e. parents first / reverse elimination order) */
  inline void push_front(const sharedConditional& conditional) {
    conditionals_.push_front(conditional);
  }

  /// push_back an entire Bayes net
  void push_back(const BayesNet<CONDITIONAL> bn);

  /// push_front an entire Bayes net
  void push_front(const BayesNet<CONDITIONAL> bn);

  /** += syntax for push_back, e.g. bayesNet += c1, c2, c3
   * @param conditional The conditional to add to the back of the BayesNet
   */
  boost::assign::list_inserter<boost::assign_detail::call_push_back<BayesNet<CONDITIONAL> >, sharedConditional>
  operator+=(const sharedConditional& conditional) {
    return boost::assign::make_list_inserter(boost::assign_detail::call_push_back<BayesNet<CONDITIONAL> >(*this))(conditional); }

  /**
   * pop_front: remove node at the bottom, used in marginalization
   * For example P(ABC)=P(A|BC)P(B|C)P(C) becomes P(BC)=P(B|C)P(C)
   */
  void pop_front() {conditionals_.pop_front();}

  /** Permute the variables in the BayesNet */
  void permuteWithInverse(const Permutation& inversePermutation);

  /**
   * Permute the variables when only separator variables need to be permuted.
   * Returns true if any reordered variables appeared in the separator and
   * false if not.
   */
  bool permuteSeparatorWithInverse(const Permutation& inversePermutation);

  iterator begin()          {return conditionals_.begin();}		///<TODO: comment
  iterator end()            {return conditionals_.end();}			///<TODO: comment
  reverse_iterator rbegin() {return conditionals_.rbegin();}	///<TODO: comment
  reverse_iterator rend()   {return conditionals_.rend();}	  ///<TODO: comment

  /** saves the bayes to a text file in GraphViz format */
  //		void saveGraph(const std::string& s) const;

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(conditionals_);
  }

	/// @}
}; // BayesNet

} // namespace gtsam

#include <gtsam/inference/BayesNet-inl.h>

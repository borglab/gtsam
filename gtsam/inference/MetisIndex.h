/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file    MetisIndex.h
* @author  Andrew Melim
* @date    Oct. 10, 2014
*/

#pragma once

#include <vector>
#include <boost/foreach.hpp>

#include <gtsam/base/FastList.h>
#include <gtsam/base/types.h>
#include <gtsam/base/timing.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/3rdparty/metis/metis.h>
#include <boost/bimap.hpp>

namespace gtsam {
/**
	* The MetisIndex class converts a factor graph into the Compressed Sparse Row format for use in
	* METIS algorithms. Specifically, two vectors store the adjacency structure of the graph. It is built
	* from a factor graph prior to elimination, and stores the list of factors
	* that involve each variable.
	* \nosubgrouping
	*/
class GTSAM_EXPORT MetisIndex
{
public:
	typedef boost::shared_ptr<MetisIndex> shared_ptr;
  typedef boost::bimap<Key, idx_t>      bm_type;

private:
  FastVector<idx_t>        xadj_; // Index of node's adjacency list in adj
  FastVector<idx_t>        adj_; // Stores ajacency lists of all nodes, appended into a single vector
  FastVector<idx_t>        iadj_; // Integer keys for passing into metis. One to one mapping with adj_;
  boost::bimap<Key, idx_t> intKeyBMap_; // Stores Key <-> integer value relationship
	size_t nFactors_;        // Number of factors in the original factor graph
	size_t nKeys_;          

public:
	/// @name Standard Constructors
	/// @{

	/** Default constructor, creates empty MetisIndex */
	MetisIndex() : nFactors_(0), nKeys_(0) {}

	template<class FG>
	MetisIndex(const FG& factorGraph) : nFactors_(0), nKeys_(0) {
			augment(factorGraph); }

	~MetisIndex(){}
	/// @}
	/// @name Advanced Interface
	/// @{

	/**
	* Augment the variable index with new factors.  This can be used when
	* solving problems incrementally.
	*/
	template<class FACTOR>
	void augment(const FactorGraph<FACTOR>& factors);

  std::vector<idx_t> xadj() const { return   xadj_; }
  std::vector<idx_t>  adj() const { return    adj_; }
	size_t          nValues() const { return  nKeys_; }
  Key intToKey(idx_t value) const {
    assert(value >= 0);
    return intKeyBMap_.right.find(value)->second;
  }

	/// @}
};

}

#include <gtsam/inference/MetisIndex-inl.h>

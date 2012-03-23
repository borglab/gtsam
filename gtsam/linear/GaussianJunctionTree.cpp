/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianJunctionTree.cpp
 * @date Jul 12, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: the Gaussian junction tree
 */

#include <gtsam/inference/ClusterTree.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianBayesTree.h>

#include <vector>

#include <boost/foreach.hpp>

namespace gtsam {

  // explicit template instantiation
  template class JunctionTree<GaussianFactorGraph>;
  template class ClusterTree<GaussianFactorGraph>;

	using namespace std;

	/* ************************************************************************* */
	VectorValues GaussianJunctionTree::optimize(Eliminate function) const {
	  tic(1, "GJT eliminate");
		// eliminate from leaves to the root
		BTClique::shared_ptr rootClique(this->eliminate(function));
    toc(1, "GJT eliminate");

		// Allocate solution vector and copy RHS
    tic(2, "allocate VectorValues");
    vector<size_t> dims(rootClique->conditional()->back()+1, 0);
		countDims(rootClique, dims);
		VectorValues result(dims);
    toc(2, "allocate VectorValues");

		// back-substitution
    tic(3, "back-substitute");
		internal::optimizeInPlace<GaussianBayesTree>(rootClique, result);
    toc(3, "back-substitute");
		return result;
	}

	/* ************************************************************************* */
} //namespace gtsam

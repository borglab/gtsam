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

#include <gtsam/inference/ClusterTreeOrdered.h>
#include <gtsam/inference/JunctionTreeOrdered.h>
#include <gtsam/linear/GaussianJunctionTreeOrdered.h>
#include <gtsam/linear/GaussianBayesTreeOrdered.h>

#include <vector>

#include <boost/foreach.hpp>

namespace gtsam {

  // explicit template instantiation
  template class JunctionTreeOrdered<GaussianFactorGraphOrdered>;
  template class ClusterTreeOrdered<GaussianFactorGraphOrdered>;

  using namespace std;

  /* ************************************************************************* */
  VectorValuesOrdered GaussianJunctionTreeOrdered::optimize(Eliminate function) const {
    gttic(GJT_eliminate);
    // eliminate from leaves to the root
    BTClique::shared_ptr rootClique(this->eliminate(function));
    gttoc(GJT_eliminate);

    // Allocate solution vector and copy RHS
    gttic(allocate_VectorValues);
    vector<size_t> dims(rootClique->conditional()->back()+1, 0);
    countDims(rootClique, dims);
    VectorValuesOrdered result(dims);
    gttoc(allocate_VectorValues);

    // back-substitution
    gttic(backsubstitute);
    internal::optimizeInPlace<GaussianBayesTreeOrdered>(rootClique, result);
    gttoc(backsubstitute);
    return result;
  }

  /* ************************************************************************* */
} //namespace gtsam

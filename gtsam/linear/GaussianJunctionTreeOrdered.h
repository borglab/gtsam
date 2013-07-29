/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianJunctionTree.h
 * @date Jul 12, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: the Gaussian junction tree
 */

#pragma once

#include <boost/foreach.hpp>
#include <gtsam/inference/JunctionTreeOrdered.h>
#include <gtsam/linear/GaussianBayesTreeOrdered.h>
#include <gtsam/linear/GaussianConditionalOrdered.h>
#include <gtsam/linear/GaussianFactorGraphOrdered.h>

namespace gtsam {

  /**
   * A JunctionTree where all the factors are of type GaussianFactor.
   *
   * In GTSAM, typically, a GaussianJunctionTree is created directly from a GaussianFactorGraph,
   * after which you call optimize() to solve for the mean, or JunctionTree::eliminate() to
   * create a BayesTree<GaussianConditional>. In both cases, you need to provide a basic
   * GaussianFactorGraph::Eliminate function that will be used to
   *
   * \addtogroup Multifrontal
   */
  class GaussianJunctionTreeOrdered: public JunctionTreeOrdered<GaussianFactorGraphOrdered> {

  public:
    typedef boost::shared_ptr<GaussianJunctionTreeOrdered> shared_ptr;
    typedef JunctionTreeOrdered<GaussianFactorGraphOrdered> Base;
    typedef Base::sharedClique sharedClique;
    typedef GaussianFactorGraphOrdered::Eliminate Eliminate;

  public :

    /** Default constructor */
    GaussianJunctionTreeOrdered() : Base() {}

    /** Constructor from a factor graph.  Builds a VariableIndex. */
    GaussianJunctionTreeOrdered(const GaussianFactorGraphOrdered& fg) : Base(fg) {}

    /** Construct from a factor graph and a pre-computed variable index. */
    GaussianJunctionTreeOrdered(const GaussianFactorGraphOrdered& fg, const VariableIndexOrdered& variableIndex)
    : Base(fg, variableIndex) {}

    /**
     *  optimize the linear graph
     */
    GTSAM_EXPORT VectorValuesOrdered optimize(Eliminate function) const;

    // convenient function to return dimensions of all variables in the BayesTree<GaussianConditional>
    template<class DIM_CONTAINER, class CLIQUE>
    static void countDims(const BayesTreeOrdered<GaussianConditionalOrdered,CLIQUE>& bayesTree, DIM_CONTAINER& dims) {
      dims = DIM_CONTAINER(bayesTree.root()->conditional()->back()+1, 0);
      countDims(bayesTree.root(), dims);
    }

  private:
    template<class DIM_CONTAINER, class CLIQUE>
    static void countDims(const boost::shared_ptr<CLIQUE>& clique, DIM_CONTAINER& dims) {
      GaussianConditionalOrdered::const_iterator it = clique->conditional()->beginFrontals();
      for (; it != clique->conditional()->endFrontals(); ++it) {
        assert(dims.at(*it) == 0);
        dims.at(*it) = clique->conditional()->dim(it);
      }

      BOOST_FOREACH(const typename CLIQUE::shared_ptr& child, clique->children()) {
        countDims(child, dims);
      }
    }

  }; // GaussianJunctionTree

} // namespace gtsam

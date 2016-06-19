/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ISAM2-inl.h
 * @brief 
 * @author Richard Roberts
 * @date Mar 16, 2012
 */


#pragma once

#include <stack>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

/* ************************************************************************* */
template<class VALUE>
VALUE ISAM2::calculateEstimate(Key key) const {
  const Vector& delta = getDelta()[key];
  return traits<VALUE>::Retract(theta_.at<VALUE>(key), delta);
}

/* ************************************************************************* */
namespace internal {
template<class CLIQUE>
void optimizeWildfire(const boost::shared_ptr<CLIQUE>& clique, double threshold,
    KeySet& changed, const KeySet& replaced, VectorValues& delta, size_t& count)
{
  // if none of the variables in this clique (frontal and separator!) changed
  // significantly, then by the running intersection property, none of the
  // cliques in the children need to be processed

  // Are any clique variables part of the tree that has been redone?
  bool cliqueReplaced = replaced.exists((*clique)->frontals().front());
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
  for(Key frontal: clique->conditional()->frontals()) {
    assert(cliqueReplaced == replaced.exists(frontal));
  }
#endif

  // If not redone, then has one of the separator variables changed significantly?
  bool recalculate = cliqueReplaced;
  if(!recalculate) {
    for(Key parent: clique->conditional()->parents()) {
      if(changed.exists(parent)) {
        recalculate = true;
        break;
      }
    }
  }

  // Solve clique if it was replaced, or if any parents were changed above the
  // threshold or themselves replaced.
  if(recalculate) {

    // Temporary copy of the original values, to check how much they change
    FastVector<Vector> originalValues(clique->conditional()->nrFrontals());
    GaussianConditional::const_iterator it;
    for(it = clique->conditional()->beginFrontals(); it!=clique->conditional()->endFrontals(); it++) {
      originalValues[it - clique->conditional()->beginFrontals()] = delta[*it];
    }

    // Back-substitute
    delta.update(clique->conditional()->solve(delta));
    count += clique->conditional()->nrFrontals();

    // Whether the values changed above a threshold, or always true if the
    // clique was replaced.
    bool valuesChanged = cliqueReplaced;
    for(it = clique->conditional()->beginFrontals(); it != clique->conditional()->endFrontals(); it++) {
      if(!valuesChanged) {
        const Vector& oldValue(originalValues[it - clique->conditional()->beginFrontals()]);
        const Vector& newValue(delta[*it]);
        if((oldValue - newValue).lpNorm<Eigen::Infinity>() >= threshold) {
          valuesChanged = true;
          break;
        }
      } else
        break;
    }

    // If the values were above the threshold or this clique was replaced
    if(valuesChanged) {
      // Set changed flag for each frontal variable and leave the new values
      for(Key frontal: clique->conditional()->frontals()) {
        changed.insert(frontal);
      }
    } else {
      // Replace with the old values
      for(it = clique->conditional()->beginFrontals(); it != clique->conditional()->endFrontals(); it++) {
        delta[*it] = originalValues[it - clique->conditional()->beginFrontals()];
      }
    }

    // Recurse to children
    for(const typename CLIQUE::shared_ptr& child: clique->children) {
      optimizeWildfire(child, threshold, changed, replaced, delta, count);
    }
  }
}

template<class CLIQUE>
bool optimizeWildfireNode(const boost::shared_ptr<CLIQUE>& clique, double threshold,
    KeySet& changed, const KeySet& replaced, VectorValues& delta, size_t& count)
{
  // if none of the variables in this clique (frontal and separator!) changed
  // significantly, then by the running intersection property, none of the
  // cliques in the children need to be processed

  // Are any clique variables part of the tree that has been redone?
  bool cliqueReplaced = replaced.exists(clique->conditional()->frontals().front());
#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
  for(Key frontal: clique->conditional()->frontals()) {
    assert(cliqueReplaced == replaced.exists(frontal));
  }
#endif

  // If not redone, then has one of the separator variables changed significantly?
  bool recalculate = cliqueReplaced;
  if(!recalculate) {
    for(Key parent: clique->conditional()->parents()) {
      if(changed.exists(parent)) {
        recalculate = true;
        break;
      }
    }
  }

  // Solve clique if it was replaced, or if any parents were changed above the
  // threshold or themselves replaced.
  // TODO(gareth): This code shares a lot of logic w/ linearAlgorithms-inst, potentially refactor
  if(recalculate)
  {
    // Temporary copy of the original values, to check how much they change
    FastVector<Vector> originalValues(clique->conditional()->nrFrontals());
    GaussianConditional::const_iterator it;
    for(it = clique->conditional()->beginFrontals(); it != clique->conditional()->endFrontals(); it++) {
      originalValues[it - clique->conditional()->beginFrontals()] = delta[*it];
    }

    // Back-substitute - special version stores solution pointers in cliques for fast access.
    {
      // Create solution part pointers if necessary and possible - necessary if solnPointers_ is
      // empty, and possible if either we're a root, or we have a parent with valid solnPointers_.
      boost::shared_ptr<CLIQUE> parent = clique->parent_.lock();
      if(clique->solnPointers_.empty() && (clique->isRoot() || !parent->solnPointers_.empty()))
      {
        for(Key key: clique->conditional()->frontals())
          clique->solnPointers_.insert(std::make_pair(key, delta.find(key)));
        for(Key key: clique->conditional()->parents())
          clique->solnPointers_.insert(std::make_pair(key, parent->solnPointers_.at(key)));
      }

      // See if we can use solution part pointers - we can if they either already existed or were
      // created above.
      if(!clique->solnPointers_.empty())
      {
        GaussianConditional& c = *clique->conditional();
        // Solve matrix
        Vector xS;
        {
          // Count dimensions of vector
          DenseIndex dim = 0;
          FastVector<VectorValues::const_iterator> parentPointers;
          parentPointers.reserve(clique->conditional()->nrParents());
          for(Key parent: clique->conditional()->parents()) {
            parentPointers.push_back(clique->solnPointers_.at(parent));
            dim += parentPointers.back()->second.size();
          }

          // Fill parent vector
          xS.resize(dim);
          DenseIndex vectorPos = 0;
          for(const VectorValues::const_iterator& parentPointer: parentPointers) {
            const Vector& parentVector = parentPointer->second;
            xS.block(vectorPos,0,parentVector.size(),1) = parentVector.block(0,0,parentVector.size(),1);
            vectorPos += parentVector.size();
          }
        }

        // NOTE(gareth): We can no longer write: xS = b - S * xS
        // This is because Eigen (as of 3.3) no longer evaluates S * xS into
        // a temporary, and the operation trashes valus in xS.
        // See: http://eigen.tuxfamily.org/index.php?title=3.3
        const Vector rhs = c.getb() - c.get_S() * xS;
        const Vector solution = c.get_R().triangularView<Eigen::Upper>().solve(rhs);

        // Check for indeterminant solution
        if(solution.hasNaN()) throw IndeterminantLinearSystemException(c.keys().front());

        // Insert solution into a VectorValues
        DenseIndex vectorPosition = 0;
        for(GaussianConditional::const_iterator frontal = c.beginFrontals(); frontal != c.endFrontals(); ++frontal) {
          clique->solnPointers_.at(*frontal)->second = solution.segment(vectorPosition, c.getDim(frontal));
          vectorPosition += c.getDim(frontal);
        }
      }
      else
      {
        // Just call plain solve because we couldn't use solution pointers.
        delta.update(clique->conditional()->solve(delta));
      }
    }
    count += clique->conditional()->nrFrontals();

    // Whether the values changed above a threshold, or always true if the
    // clique was replaced.
    bool valuesChanged = cliqueReplaced;
    for(it = clique->conditional()->beginFrontals(); it != clique->conditional()->endFrontals(); it++) {
      if(!valuesChanged) {
        const Vector& oldValue(originalValues[it - clique->conditional()->beginFrontals()]);
        const Vector& newValue(delta[*it]);
        if((oldValue - newValue).lpNorm<Eigen::Infinity>() >= threshold) {
          valuesChanged = true;
          break;
        }
      } else
        break;
    }

    // If the values were above the threshold or this clique was replaced
    if(valuesChanged) {
      // Set changed flag for each frontal variable and leave the new values
      for(Key frontal: clique->conditional()->frontals()) {
        changed.insert(frontal);
      }
    } else {
      // Replace with the old values
      for(it = clique->conditional()->beginFrontals(); it != clique->conditional()->endFrontals(); it++) {
        delta[*it] = originalValues[it - clique->conditional()->beginFrontals()];
      }
    }
  }

  return recalculate;
}

} // namespace internal

/* ************************************************************************* */
template<class CLIQUE>
size_t optimizeWildfire(const boost::shared_ptr<CLIQUE>& root, double threshold, const KeySet& keys, VectorValues& delta) {
  KeySet changed;
  int count = 0;
  // starting from the root, call optimize on each conditional
  if(root)
    internal::optimizeWildfire(root, threshold, changed, keys, delta, count);
  return count;
}

/* ************************************************************************* */
template<class CLIQUE>
size_t optimizeWildfireNonRecursive(const boost::shared_ptr<CLIQUE>& root, double threshold, const KeySet& keys, VectorValues& delta)
{
  KeySet changed;
  size_t count = 0;

  if (root) {
    std::stack<boost::shared_ptr<CLIQUE> > travStack;
    travStack.push(root);
    boost::shared_ptr<CLIQUE> currentNode = root;
    while (!travStack.empty()) {
      currentNode = travStack.top();
      travStack.pop();
      bool recalculate = internal::optimizeWildfireNode(currentNode, threshold, changed, keys, delta, count);
      if (recalculate) {
        for(const typename CLIQUE::shared_ptr& child: currentNode->children) {
          travStack.push(child);
        }
      }
    }
  }

  return count;
}

/* ************************************************************************* */
template<class CLIQUE>
void nnz_internal(const boost::shared_ptr<CLIQUE>& clique, int& result) {
  int dimR = (int)clique->conditional()->rows();
  int dimSep = (int)clique->conditional()->get_S().cols();
  result += ((dimR+1)*dimR)/2 + dimSep*dimR;
  // traverse the children
  for(const typename CLIQUE::shared_ptr& child: clique->children) {
    nnz_internal(child, result);
  }
}

/* ************************************************************************* */
template<class CLIQUE>
int calculate_nnz(const boost::shared_ptr<CLIQUE>& clique) {
  int result = 0;
  // starting from the root, add up entries of frontal and conditional matrices of each conditional
  nnz_internal(clique, result);
  return result;
}

}


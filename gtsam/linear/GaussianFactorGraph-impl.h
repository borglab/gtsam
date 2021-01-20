/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactorGraph.cpp
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 * @author  Richard Roberts
 * @author  Gerry Chen
 * @author  Frank Dellaert
 */

#include <gtsam/linear/GaussianFactorGraph.h>  // for autocomplete/intellisense

namespace gtsam {

/* ************************************************************************* */
template <typename T>
void GaussianFactorGraph::sparseJacobianInPlace(T& entries,
                                                const Ordering& ordering,
                                                size_t& nrows,
                                                size_t& ncols) const {
  gttic_(GaussianFactorGraph_sparseJacobianInPlace);
  // First find dimensions of each variable
  typedef std::map<Key, size_t> KeySizeMap;
  KeySizeMap dims;
  for (const sharedFactor& factor : *this) {
    if (!static_cast<bool>(factor))
      continue;

    for (auto it = factor->begin(); it != factor->end(); ++it) {
      dims[*it] = factor->getDim(it);
    }
  }

  // Compute first scalar column of each variable
  ncols = 0;
  KeySizeMap columnIndices = dims;
  for (const auto key : ordering) {
    columnIndices[key] = ncols;
    ncols += dims[key];
  }

  // Iterate over all factors, adding sparse scalar entries
  nrows = 0;
  for (const sharedFactor& factor : *this) {
    if (!static_cast<bool>(factor)) continue;

    // Convert to JacobianFactor if necessary
    JacobianFactor::shared_ptr jacobianFactor(
        boost::dynamic_pointer_cast<JacobianFactor>(factor));
    if (!jacobianFactor) {
      HessianFactor::shared_ptr hessian(
          boost::dynamic_pointer_cast<HessianFactor>(factor));
      if (hessian)
        jacobianFactor.reset(new JacobianFactor(*hessian));
      else
        throw std::invalid_argument(
            "GaussianFactorGraph contains a factor that is neither a JacobianFactor nor a HessianFactor.");
    }

    // Whiten the factor and add entries for it
    // iterate over all variables in the factor
    const JacobianFactor whitened(jacobianFactor->whiten());
    for (JacobianFactor::const_iterator key = whitened.begin();
        key < whitened.end(); ++key) {
      JacobianFactor::constABlock whitenedA = whitened.getA(key);
      // find first column index for this key
      size_t column_start = columnIndices[*key];
      for (size_t i = 0; i < (size_t) whitenedA.rows(); i++)
        for (size_t j = 0; j < (size_t) whitenedA.cols(); j++) {
          double s = whitenedA(i, j);
          if (std::abs(s) > 1e-12)
            entries.emplace_back(nrows + i, column_start + j, s);
        }
    }

    JacobianFactor::constBVector whitenedb(whitened.getb());
    for (size_t i = 0; i < (size_t) whitenedb.size(); i++) {
      double s = whitenedb(i);
      if (std::abs(s) > 1e-12) entries.emplace_back(nrows + i, ncols, s);
    }

    // Increment row index
    nrows += jacobianFactor->rows();
  }

  ncols++;  // +1 for b-column
}

}  // namespace gtsam

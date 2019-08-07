/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2UpdateParams.h
 * @brief   Class that stores extra params for ISAM2::update()
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert, Jose Luis Blanco
 */

// \callgraph

#pragma once

#include <gtsam/base/FastList.h>
#include <gtsam/dllexport.h>              // GTSAM_EXPORT
#include <gtsam/inference/Key.h>          // Key, KeySet
#include <gtsam/nonlinear/ISAM2Result.h>  //FactorIndices
#include <boost/optional.hpp>

namespace gtsam {

/**
 * @addtogroup ISAM2
 * This struct is used by ISAM2::update() to pass additional parameters to
 * give the user a fine-grained control on how factors and relinearized, etc.
 */
struct ISAM2UpdateParams {
  ISAM2UpdateParams() = default;

  /** Indices of factors to remove from system (default: empty) */
  FactorIndices removeFactorIndices;

  /** An optional map of keys to group labels, such that a variable can be
   * constrained to a particular grouping in the BayesTree */
  boost::optional<FastMap<Key, int>> constrainedKeys{boost::none};

  /** An optional set of nonlinear keys that iSAM2 will hold at a constant
   * linearization point, regardless of the size of the linear delta */
  boost::optional<FastList<Key>> noRelinKeys{boost::none};

  /** An optional set of nonlinear keys that iSAM2 will re-eliminate, regardless
   * of the size of the linear delta. This allows the provided keys to be
   * reordered. */
  boost::optional<FastList<Key>> extraReelimKeys{boost::none};

  /** Relinearize any variables whose delta magnitude is sufficiently large
   * (Params::relinearizeThreshold), regardless of the relinearization
   * interval (Params::relinearizeSkip). */
  bool force_relinearize{false};

  /** An optional set of new Keys that are now affected by factors,
   * indexed by factor indices (as returned by ISAM2::update()).
   * Use when working with smart factors. For example:
   *  - Timestamp `i`: ISAM2::update() called with a new smart factor depending
   *    on Keys `X(0)` and `X(1)`. It returns that the factor index for the new
   *    smart factor (inside ISAM2) is `13`.
   *  - Timestamp `i+1`: The same smart factor has been augmented to now also
   *    depend on Keys `X(2)`, `X(3)`. Next call to ISAM2::update() must include
   *    its `newAffectedKeys` field with the map `13 -> {X(2), X(3)}`.
   */
  boost::optional<FastMap<FactorIndex, KeySet>> newAffectedKeys{boost::none};

  /** By default, iSAM2 uses a wildfire update scheme that stops updating when
   * the deltas become too small down in the tree. This flagg forces a full
   * solve instead. */
  bool forceFullSolve{false};
};

}  // namespace gtsam

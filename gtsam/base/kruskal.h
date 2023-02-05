/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphBuilder-inl.h
 * @date Dec 31, 2009
 * @author Frank Dellaert, Yong-Dian Jian
 */

#pragma once

#include <gtsam/base/FastMap.h>

#include <vector>

namespace gtsam::utils {
template <class FactorGraph>
std::vector<size_t> kruskal(const FactorGraph &fg,
                            const std::vector<double> &weights);
}

#include <gtsam/base/kruskal-inl.h>
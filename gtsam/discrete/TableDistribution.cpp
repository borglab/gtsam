/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file TableDistribution.cpp
 *  @date Dec 22, 2024
 *  @author Varun Agrawal
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/utilities.h>
#include <gtsam/discrete/Ring.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/discrete/TableDistribution.h>
#include <gtsam/hybrid/HybridValues.h>

#include <algorithm>
#include <cassert>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace std;
using std::pair;
using std::stringstream;
using std::vector;
namespace gtsam {

/// Normalize sparse_table
static SparseVector normalizeSparseTable(
    const SparseVector& sparse_table) {
  return sparse_table / sparse_table.sum();
}

/* ************************************************************************** */
TableDistribution::TableDistribution(const TableFactor& f)
    : BaseConditional(f.keys().size(), f.discreteKeys(),
                      f / f.sum(f.keys().size())) {}

/* ************************************************************************** */
TableDistribution::TableDistribution(const DiscreteKeys& keys,
                                     const std::vector<double>& potentials)
    : BaseConditional(keys.size(), keys,
                      std::make_shared<TableFactor>(
                          keys, normalizeSparseTable(
                                    TableFactor::Convert(keys, potentials)))) {}

/* ************************************************************************** */
TableDistribution::TableDistribution(const DiscreteKeys& keys,
                                     const std::string& potentials)
    : BaseConditional(keys.size(), keys,
                      std::make_shared<TableFactor>(
                          keys, normalizeSparseTable(
                                    TableFactor::Convert(keys, potentials)))) {}

/* ************************************************************************** */
bool TableDistribution::equals(const DiscreteFactor& other, double tol) const {
  auto dtc = dynamic_cast<const TableDistribution*>(&other);
  if (!dtc) {
    return false;
  } else {
    return tableFactor().equals(dtc->tableFactor(), tol) &&
           DiscreteConditional::BaseConditional::equals(*dtc, tol);
  }
}

/* ************************************************************************ */
DiscreteValues TableDistribution::argmax() const {
  uint64_t maxIdx = 0;
  double maxValue = 0.0;

  const SparseVector& sparseTable = tableFactor().sparseTable();

  for (SparseIt it(sparseTable); it; ++it) {
    if (it.value() > maxValue) {
      maxIdx = it.index();
      maxValue = it.value();
    }
  }

  return tableFactor().findAssignments(maxIdx);
}

/* ****************************************************************************/
DiscreteFactor::shared_ptr TableDistribution::prune(
    size_t maxNrAssignments) const {
  return std::make_shared<TableDistribution>(
      static_cast<const TableFactor&>(*factor_->prune(maxNrAssignments)));
}

}  // namespace gtsam

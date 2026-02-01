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
static Eigen::SparseVector<double> normalizeSparseTable(
    const Eigen::SparseVector<double>& sparse_table) {
  return sparse_table / sparse_table.sum();
}

/* ************************************************************************** */
TableDistribution::TableDistribution(const TableFactor& f)
    : BaseConditional(f.keys().size(), f.discreteKeys(), ADT(nullptr)),
      table_(f / (*std::dynamic_pointer_cast<TableFactor>(
                     f.sum(f.keys().size())))) {}

/* ************************************************************************** */
TableDistribution::TableDistribution(const DiscreteKeys& keys,
                                     const std::vector<double>& potentials)
    : BaseConditional(keys.size(), keys, ADT(nullptr)),
      table_(TableFactor(
          keys, normalizeSparseTable(TableFactor::Convert(keys, potentials)))) {
}

/* ************************************************************************** */
TableDistribution::TableDistribution(const DiscreteKeys& keys,
                                     const std::string& potentials)
    : BaseConditional(keys.size(), keys, ADT(nullptr)),
      table_(TableFactor(
          keys, normalizeSparseTable(TableFactor::Convert(keys, potentials)))) {
}

/* ************************************************************************** */
void TableDistribution::print(const string& s,
                              const KeyFormatter& formatter) const {
  cout << s << " P( ";
  for (const_iterator it = beginFrontals(); it != endFrontals(); ++it) {
    cout << formatter(*it) << " ";
  }
  cout << "):\n";
  table_.print("", formatter);
  cout << endl;
}

/* ************************************************************************** */
bool TableDistribution::equals(const DiscreteFactor& other, double tol) const {
  auto dtc = dynamic_cast<const TableDistribution*>(&other);
  if (!dtc) {
    return false;
  } else {
    const DiscreteConditional& f(
        static_cast<const DiscreteConditional&>(other));
    return table_.equals(dtc->table_, tol) &&
           DiscreteConditional::BaseConditional::equals(f, tol);
  }
}

/* ****************************************************************************/
DiscreteFactor::shared_ptr TableDistribution::sum(size_t nrFrontals) const {
  return table_.sum(nrFrontals);
}

/* ****************************************************************************/
DiscreteFactor::shared_ptr TableDistribution::sum(const Ordering& keys) const {
  return table_.sum(keys);
}

/* ****************************************************************************/
DiscreteFactor::shared_ptr TableDistribution::max(size_t nrFrontals) const {
  return table_.max(nrFrontals);
}

/* ****************************************************************************/
DiscreteFactor::shared_ptr TableDistribution::max(const Ordering& keys) const {
  return table_.max(keys);
}

/* ****************************************************************************/
DiscreteFactor::shared_ptr TableDistribution::operator*(double s) const {
  return table_ * s;
}

/* ****************************************************************************/
DiscreteFactor::shared_ptr TableDistribution::operator/(
    const DiscreteFactor::shared_ptr& f) const {
  return table_ / f;
}

/* ************************************************************************ */
DiscreteValues TableDistribution::argmax() const {
  uint64_t maxIdx = 0;
  double maxValue = 0.0;

  Eigen::SparseVector<double> sparseTable = table_.sparseTable();

  for (SparseIt it(sparseTable); it; ++it) {
    if (it.value() > maxValue) {
      maxIdx = it.index();
      maxValue = it.value();
    }
  }

  return table_.findAssignments(maxIdx);
}

/* ****************************************************************************/
void TableDistribution::prune(size_t maxNrAssignments) {
  table_ = table_.prune(maxNrAssignments);
}

/* ****************************************************************************/
size_t TableDistribution::sample(const DiscreteValues& parentsValues,
                                 std::mt19937_64* rng) const {
  DiscreteKeys parentsKeys;
  for (auto&& [key, _] : parentsValues) {
    parentsKeys.push_back({key, table_.cardinality(key)});
  }

  // Get the correct conditional distribution: P(F|S=parentsValues)
  TableFactor pFS = table_.choose(parentsValues, parentsKeys);

  // TODO(Duy): only works for one key now, seems horribly slow this way
  if (nrFrontals() != 1) {
    throw std::invalid_argument(
        "TableDistribution::sample can only be called on single variable "
        "conditionals");
  }
  Key key = firstFrontalKey();
  size_t nj = cardinality(key);
  vector<double> p(nj);
  DiscreteValues frontals;
  for (size_t value = 0; value < nj; value++) {
    frontals[key] = value;
    p[value] = pFS(frontals);  // P(F=value|S=parentsValues)
    if (p[value] == 1.0) {
      return value;  // shortcut exit
    }
  }

  // Check if rng is nullptr, then assign default
  rng = (rng == nullptr) ? &kRandomNumberGenerator : rng;

  std::discrete_distribution<size_t> distribution(p.begin(), p.end());
  return distribution(*rng);
}

}  // namespace gtsam

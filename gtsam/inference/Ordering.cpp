/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.cpp
 * @author  Richard Roberts
 * @author  Andrew Melim
 * @date    Sep 2, 2010
 */

#include <vector>
#include <limits>

#include <gtsam/inference/Ordering.h>
#include <gtsam/3rdparty/CCOLAMD/Include/ccolamd.h>

#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
#include <metis.h>
#endif

using namespace std;

namespace gtsam {

/* ************************************************************************* */
FastMap<Key, size_t> Ordering::invert() const {
  FastMap<Key, size_t> inverted;
  for (size_t pos = 0; pos < this->size(); ++pos)
    inverted.insert(make_pair((*this)[pos], pos));
  return inverted;
}

/* ************************************************************************* */
Ordering Ordering::Colamd(const VariableIndex& variableIndex) {
  // Call constrained version with all groups set to zero
  vector<int> dummy_groups(variableIndex.size(), 0);
  return Ordering::ColamdConstrained(variableIndex, dummy_groups);
}

/* ************************************************************************* */
Ordering Ordering::ColamdConstrained(const VariableIndex& variableIndex,
    std::vector<int>& cmember) {
  gttic(Ordering_COLAMDConstrained);

  gttic(Prepare);
  const size_t nVars = variableIndex.size();
  if (nVars == 0)
  {
    return Ordering();
  }

  if (nVars == 1)
  {
    return Ordering(KeyVector(1, variableIndex.begin()->first));
  }

  const size_t nEntries = variableIndex.nEntries(), nFactors =
      variableIndex.nFactors();
  // Convert to compressed column major format colamd wants it in (== MATLAB format!)
  const size_t Alen = ccolamd_recommended((int) nEntries, (int) nFactors,
      (int) nVars); /* colamd arg 3: size of the array A */
  vector<int> A = vector<int>(Alen); /* colamd arg 4: row indices of A, of size Alen */
  vector<int> p = vector<int>(nVars + 1); /* colamd arg 5: column pointers of A, of size n_col+1 */

  // Fill in input data for COLAMD
  p[0] = 0;
  int count = 0;
  KeyVector keys(nVars); // Array to store the keys in the order we add them so we can retrieve them in permuted order
  size_t index = 0;
  for (auto key_factors: variableIndex) {
    // Arrange factor indices into COLAMD format
    const FactorIndices& column = key_factors.second;
    for(size_t factorIndex: column) {
      A[count++] = (int) factorIndex; // copy sparse column
    }
    p[index + 1] = count;  // column j (base 1) goes from A[j-1] to A[j]-1
    // Store key in array and increment index
    keys[index] = key_factors.first;
    ++index;
  }

  assert((size_t)count == variableIndex.nEntries());

  //double* knobs = nullptr; /* colamd arg 6: parameters (uses defaults if nullptr) */
  double knobs[CCOLAMD_KNOBS];
  ccolamd_set_defaults(knobs);
  knobs[CCOLAMD_DENSE_ROW] = -1;
  knobs[CCOLAMD_DENSE_COL] = -1;

  int stats[CCOLAMD_STATS]; /* colamd arg 7: colamd output statistics and error codes */

  gttoc(Prepare);

  // call colamd, result will be in p
  /* returns (1) if successful, (0) otherwise*/
  if (nVars > 0) {
    gttic(ccolamd);
    int rv = ccolamd((int) nFactors, (int) nVars, (int) Alen, &A[0], &p[0],
        knobs, stats, &cmember[0]);
    if (rv != 1) {
      throw runtime_error("ccolamd failed with return value " + to_string(rv));
    }
  }

  //  ccolamd_report(stats);

  // Convert elimination ordering in p to an ordering
  gttic(Fill_Ordering);
  Ordering result;
  result.resize(nVars);
  for (size_t j = 0; j < nVars; ++j)
    result[j] = keys[p[j]];
  gttoc(Fill_Ordering);

  return result;
}

/* ************************************************************************* */
Ordering Ordering::ColamdConstrainedLast(const VariableIndex& variableIndex,
    const KeyVector& constrainLast, bool forceOrder) {
  gttic(Ordering_COLAMDConstrainedLast);

  size_t n = variableIndex.size();
  std::vector<int> cmember(n, 0);

  // Build a mapping to look up sorted Key indices by Key
  // TODO(frank): think of a way to not build this
  FastMap<Key, size_t> keyIndices;
  size_t j = 0;
  for (auto key_factors: variableIndex)
    keyIndices.insert(keyIndices.end(), make_pair(key_factors.first, j++));

  // If at least some variables are not constrained to be last, constrain the
  // ones that should be constrained.
  int group = (constrainLast.size() != n ? 1 : 0);
  for (Key key: constrainLast) {
    cmember[keyIndices.at(key)] = group;
    if (forceOrder)
      ++group;
  }

  return Ordering::ColamdConstrained(variableIndex, cmember);
}

/* ************************************************************************* */
Ordering Ordering::ColamdConstrainedFirst(const VariableIndex& variableIndex,
    const KeyVector& constrainFirst, bool forceOrder) {
  gttic(Ordering_COLAMDConstrainedFirst);

  const int none = -1;
  size_t n = variableIndex.size();
  std::vector<int> cmember(n, none);

  // Build a mapping to look up sorted Key indices by Key
  FastMap<Key, size_t> keyIndices;
  size_t j = 0;
  for (auto key_factors: variableIndex)
    keyIndices.insert(keyIndices.end(), make_pair(key_factors.first, j++));

  // If at least some variables are not constrained to be last, constrain the
  // ones that should be constrained.
  int group = 0;
  for (Key key: constrainFirst) {
    cmember[keyIndices.at(key)] = group;
    if (forceOrder)
      ++group;
  }

  if (!forceOrder && !constrainFirst.empty())
    ++group;
  for(int& c: cmember)
    if (c == none)
      c = group;

  return Ordering::ColamdConstrained(variableIndex, cmember);
}

/* ************************************************************************* */
Ordering Ordering::ColamdConstrained(const VariableIndex& variableIndex,
    const FastMap<Key, int>& groups) {
  gttic(Ordering_COLAMDConstrained);
  size_t n = variableIndex.size();
  std::vector<int> cmember(n, 0);

  // Build a mapping to look up sorted Key indices by Key
  FastMap<Key, size_t> keyIndices;
  size_t j = 0;
  for (auto key_factors: variableIndex)
    keyIndices.insert(keyIndices.end(), make_pair(key_factors.first, j++));

  // Assign groups
  typedef FastMap<Key, int>::value_type key_group;
  for(const key_group& p: groups) {
    // FIXME: check that no groups are skipped
    cmember[keyIndices.at(p.first)] = p.second;
  }

  return Ordering::ColamdConstrained(variableIndex, cmember);
}

/* ************************************************************************* */
Ordering Ordering::Metis(const MetisIndex& met) {
#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
  gttic(Ordering_METIS);

  idx_t size = met.nValues();
  if (size == 0)
  {
    return Ordering();
  }

  if (size == 1)
  {
    return Ordering(KeyVector(1, met.intToKey(0)));
  }

  vector<idx_t> xadj = met.xadj();
  vector<idx_t> adj = met.adj();
  vector<idx_t> perm, iperm;

  for (idx_t i = 0; i < size; i++) {
    perm.push_back(0);
    iperm.push_back(0);
  }

  int outputError;

  outputError = METIS_NodeND(&size, &xadj[0], &adj[0], nullptr, nullptr, &perm[0],
      &iperm[0]);
  Ordering result;

  if (outputError != METIS_OK) {
    std::cout << "METIS failed during Nested Dissection ordering!\n";
    return result;
  }

  result.resize(size);
  for (size_t j = 0; j < (size_t) size; ++j) {
    // We have to add the minKey value back to obtain the original key in the Values
    result[j] = met.intToKey(perm[j]);
  }
  return result;
#else
  throw runtime_error("GTSAM was built without support for Metis-based "
                      "nested dissection");
#endif
}

/* ************************************************************************* */
void Ordering::print(const std::string& str,
    const KeyFormatter& keyFormatter) const {
  cout << str;
  // Print ordering in index order
  // Print the ordering with varsPerLine ordering entries printed on each line,
  // for compactness.
  static const size_t varsPerLine = 10;
  bool endedOnNewline = false;
  for (size_t i = 0; i < size(); ++i) {
    if (i % varsPerLine == 0)
      cout << "Position " << i << ": ";
    if (i % varsPerLine != 0)
      cout << ", ";
    cout << keyFormatter(at(i));
    if (i % varsPerLine == varsPerLine - 1) {
      cout << "\n";
      endedOnNewline = true;
    } else {
      endedOnNewline = false;
    }
  }
  if (!endedOnNewline)
    cout << "\n";
  cout.flush();
}

/* ************************************************************************* */
Ordering::This& Ordering::operator+=(Key key) {
  this->push_back(key);
  return *this;
}

/* ************************************************************************* */
Ordering::This& Ordering::operator,(Key key) {
  this->push_back(key);
  return *this;
}

/* ************************************************************************* */
Ordering::This& Ordering::operator+=(KeyVector& keys) {
  this->insert(this->end(), keys.begin(), keys.end());
  return *this;
}

/* ************************************************************************* */
bool Ordering::contains(const Key& key) const {
  return std::find(this->begin(), this->end(), key) != this->end();
}

/* ************************************************************************* */
bool Ordering::equals(const Ordering& other, double tol) const {
  return (*this) == other;
}

}

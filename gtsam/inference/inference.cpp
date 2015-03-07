/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   inference.cpp
 * @brief  inference definitions
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/inference.h>
#include <gtsam/inference/SymbolicFactorGraph.h>

#include <boost/format.hpp>
#include <stdexcept>
#include <iostream>
#include <vector>

#include <ccolamd.h>

using namespace std;

namespace gtsam {
namespace inference {

/* ************************************************************************* */
Permutation::shared_ptr PermutationCOLAMD_(const VariableIndex& variableIndex, std::vector<int>& cmember) {
  size_t nEntries = variableIndex.nEntries(), nFactors = variableIndex.nFactors(), nVars = variableIndex.size();
  // Convert to compressed column major format colamd wants it in (== MATLAB format!)
  int Alen = ccolamd_recommended(nEntries, nFactors, nVars); /* colamd arg 3: size of the array A */
  vector<int> A = vector<int>(Alen); /* colamd arg 4: row indices of A, of size Alen */
  vector<int> p = vector<int>(nVars + 1); /* colamd arg 5: column pointers of A, of size n_col+1 */

  static const bool debug = false;

  p[0] = 0;
  int count = 0;
  for(Index var = 0; var < variableIndex.size(); ++var) {
    const VariableIndex::Factors& column(variableIndex[var]);
    size_t lastFactorId = numeric_limits<size_t>::max();
    BOOST_FOREACH(const size_t& factorIndex, column) {
      if(lastFactorId != numeric_limits<size_t>::max())
        assert(factorIndex > lastFactorId);
      A[count++] = factorIndex; // copy sparse column
      if(debug) cout << "A[" << count-1 << "] = " << factorIndex << endl;
    }
    p[var+1] = count; // column j (base 1) goes from A[j-1] to A[j]-1
  }

  assert((size_t)count == variableIndex.nEntries());

  if(debug)
    for(size_t i=0; i<nVars+1; ++i)
      cout << "p[" << i << "] = " << p[i] << endl;

  //double* knobs = NULL; /* colamd arg 6: parameters (uses defaults if NULL) */
  double knobs[CCOLAMD_KNOBS];
  ccolamd_set_defaults(knobs);
  knobs[CCOLAMD_DENSE_ROW]=-1;
  knobs[CCOLAMD_DENSE_COL]=-1;

  int stats[CCOLAMD_STATS]; /* colamd arg 7: colamd output statistics and error codes */

  // call colamd, result will be in p
  /* returns (1) if successful, (0) otherwise*/
  if(nVars > 0) {
    int rv = ccolamd(nFactors, nVars, Alen, &A[0], &p[0], knobs, stats, &cmember[0]);
    if(rv != 1)
      throw runtime_error((boost::format("ccolamd failed with return value %1%")%rv).str());
  }

  //  ccolamd_report(stats);

  // Convert elimination ordering in p to an ordering
  Permutation::shared_ptr permutation(new Permutation(nVars));
  for (Index j = 0; j < nVars; j++) {
    //    if(p[j] == -1)
    //      permutation->operator[](j) = j;
    //    else
    permutation->operator[](j) = p[j];
    if(debug) cout << "COLAMD:  " << j << "->" << p[j] << endl;
  }
  if(debug) cout << "COLAMD:  p[" << nVars << "] = " << p[nVars] << endl;

  return permutation;
}

/* ************************************************************************* */
} // \namespace inference
} // \namespace gtsam

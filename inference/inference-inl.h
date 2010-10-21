/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   inference-inl.h
 * @brief  inference template definitions
 * @author Frank Dellaert, Richard Roberts
 */

#pragma once

#include <limits>
#include <map>
#include <stdexcept>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/pool/pool_alloc.hpp>

#include <ccolamd.h>

#include <gtsam/base/timing.h>
#include <gtsam/inference/inference.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/Conditional.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
template<class VARIABLEINDEXTYPE, typename CONSTRAINTCONTAINER>
Permutation::shared_ptr Inference::PermutationCOLAMD(const VARIABLEINDEXTYPE& variableIndex, const CONSTRAINTCONTAINER& constrainLast) {
  size_t nEntries = variableIndex.nEntries(), nFactors = variableIndex.nFactors(), nVars = variableIndex.size();
  // Convert to compressed column major format colamd wants it in (== MATLAB format!)
  int Alen = ccolamd_recommended(nEntries, nFactors, nVars); /* colamd arg 3: size of the array A */
  int * A = new int[Alen]; /* colamd arg 4: row indices of A, of size Alen */
  int * p = new int[nVars + 1]; /* colamd arg 5: column pointers of A, of size n_col+1 */
  int * cmember = new int[nVars]; /* Constraint set of A, of size n_col */

  static const bool debug = false;

  p[0] = 0;
  int count = 0;
  for(Index var = 0; var < variableIndex.size(); ++var) {
    const typename VARIABLEINDEXTYPE::mapped_type& column(variableIndex[var]);
    size_t lastFactorId = numeric_limits<size_t>::max();
    BOOST_FOREACH(const typename VARIABLEINDEXTYPE::mapped_factor_type& factor_pos, column) {
      if(lastFactorId != numeric_limits<size_t>::max())
        assert(factor_pos.factorIndex > lastFactorId);
      A[count++] = factor_pos.factorIndex; // copy sparse column
      if(debug) cout << "A[" << count-1 << "] = " << factor_pos.factorIndex << endl;
    }
    p[var+1] = count; // column j (base 1) goes from A[j-1] to A[j]-1
    cmember[var] = 0;
  }

  // If at least some variables are not constrained to be last, constrain the
  // ones that should be constrained.
  if(constrainLast.size() < variableIndex.size()) {
    BOOST_FOREACH(Index var, constrainLast) {
      assert(var < nVars);
      cmember[var] = 1;
    }
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
  int rv = ccolamd(nFactors, nVars, Alen, A, p, knobs, stats, cmember);
  if(rv != 1)
    throw runtime_error((boost::format("ccolamd failed with return value %1%")%rv).str());
  delete[] A; // delete symbolic A
  delete[] cmember;

  // Convert elimination ordering in p to an ordering
  Permutation::shared_ptr permutation(new Permutation(nVars));
  for (Index j = 0; j < nVars; j++) {
    permutation->operator[](j) = p[j];
    if(debug) cout << "COLAMD:  " << j << "->" << p[j] << endl;
  }
  if(debug) cout << "COLAMD:  p[" << nVars << "] = " << p[nVars] << endl;
  delete[] p; // delete colamd result vector

  return permutation;
}

} // namespace gtsam

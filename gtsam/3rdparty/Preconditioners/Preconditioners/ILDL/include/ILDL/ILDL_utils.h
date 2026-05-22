/** This file provides several convenient utility functions for working with the
 * SymILDL library.
 *
 * Copyright (C) 2019 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include "Preconditioners/Types.h"
#include <vector>

namespace Preconditioners {
/** Given a SYMMETRIC, ROW-MAJOR sparse Eigen matrix S, this function constructs
 * and returns the compressed sparse row (CSR) representation of S.  Note that
 * since S is assumed to be symmetric, only the UPPER TRIANGLE of S is
 * referenced.
 */
void toCSR(const SparseMatrix &S, std::vector<int> &row_ptr,
           std::vector<int> &col_idx, std::vector<Scalar> &val);

} // namespace Preconditioners

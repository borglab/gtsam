#include "ILDL/ILDL_utils.h"

namespace Preconditioners {

void toCSR(const SparseMatrix &S, std::vector<int> &row_ptr,
           std::vector<int> &col_idx, std::vector<Scalar> &val) {

  size_t n = S.rows();
  size_t nnz = S.nonZeros();

  // Preallocate storage for the vectors containing the CSR representation of S
  row_ptr.resize(n + 1);
  col_idx.clear();
  col_idx.reserve(nnz);
  val.clear();
  val.reserve(nnz);

  size_t idx = 0;
  for (size_t r = 0; r < S.outerSize(); ++r) {
    // Store starting index for the for the current (rth) row.
    row_ptr[r] = idx;

    for (SparseMatrix::InnerIterator it(S, r); it; ++it) {
      // Check whether the current element belongs to the upper triangle of S
      if (it.col() >= r) {
        // This element belongs to the upper triangle of S
        col_idx.emplace_back(it.col());
        val.emplace_back(it.value());
        ++idx;
      }
    }
  }

  // Don't forget the last element of row_ptr!
  row_ptr[S.rows()] = idx;
}

} // namespace Preconditioners

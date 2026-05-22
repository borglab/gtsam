//-*- mode: c++ -*-
#ifndef _LIL_MATRIX_TO_STRING_H_
#define _LIL_MATRIX_TO_STRING_H_

#include <string>
#include <sstream>

template <class el_type>
std::string lilc_matrix<el_type> :: to_string() const
{
  std::ostringstream os;
#ifdef SYM_ILDL_DEBUG
  os << "List of Lists Matrix (" << m_n_rows << ", " << m_n_cols << ", " << nnz() << ")" << std::endl;
  
  for (int i = 0; i < n_cols(); i++) {
    os << "Column " << i << ":" << std::endl;
    os << "Row      Indices = "  << m_idx[i] << std::endl;
    os << "Non-zero Values  = "  << m_x[i]       << std::endl;
    os << std::endl;
  }
#endif
  return os.str();
}

#endif // _LIL_MATRIX_TO_STRING_H_

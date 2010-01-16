/*
 * TupleConfig-inl.h
 *
 *  Created on: Jan 14, 2010
 *      Author: richard
 */

#include "LieConfig-inl.h"

#include "TupleConfig.h"

#define INSTANTIATE_PAIR_CONFIG(J1,X1,J2,X2) \
  /*INSTANTIATE_LIE_CONFIG(J1,X1);*/ \
  /*INSTANTIATE_LIE_CONFIG(J2,X2);*/ \
  template class PairConfig<J1,X1,J2,X2>; \
  /*template void PairConfig<J1,X1,J2,X2>::print(const std::string&) const;*/ \
  template PairConfig<J1,X1,J2,X2> expmap(PairConfig<J1,X1,J2,X2>, const VectorConfig&);

namespace gtsam {

  template<class J1, class X1, class J2, class X2>
  void PairConfig<J1,X1,J2,X2>::print(const std::string& s) const {
    std::cout << "TupleConfig " << s << ", size " << size_ << "\n";
    first.print(s + "Config1: ");
    second.print(s + "Config2: ");
  }

}

/*
 * TupleConfig-inl.h
 *
 *  Created on: Jan 14, 2010
 *      Author: richard
 */

#pragma once

#include "LieConfig-inl.h"

#include "TupleConfig.h"

#define INSTANTIATE_PAIR_CONFIG(J1,X1,J2,X2) \
  /*INSTANTIATE_LIE_CONFIG(J1,X1);*/ \
  /*INSTANTIATE_LIE_CONFIG(J2,X2);*/ \
  template class PairConfig<J1,X1,J2,X2>; \
  /*template void PairConfig<J1,X1,J2,X2>::print(const std::string&) const;*/ \
  template PairConfig<J1,X1,J2,X2> expmap(PairConfig<J1,X1,J2,X2>, const VectorConfig&);

namespace gtsam {

/** PairConfig implementations */
/* ************************************************************************* */
template<class J1, class X1, class J2, class X2>
void PairConfig<J1,X1,J2,X2>::print(const std::string& s) const {
	std::cout << "TupleConfig " << s << ", size " << size_ << "\n";
	first().print(s + "Config1: ");
	second().print(s + "Config2: ");
}

/* ************************************************************************* */
template<class J1, class X1, class J2, class X2>
void PairConfig<J1,X1,J2,X2>::insert(const PairConfig& config) {
	for (typename Config1::const_iterator it = config.first().begin(); it!=config.first().end(); it++) {
		insert(it->first, it->second);
	}
	for (typename Config2::const_iterator it = config.second().begin(); it!=config.second().end(); it++) {
		insert(it->first, it->second);
	}
}

/** TupleConfigN Implementations */
/* ************************************************************************* */
template<class Config1, class Config2>
TupleConfig2<Config1, Config2>::TupleConfig2(const TupleConfig2<Config1, Config2>& config) :
		  TupleConfig<Config1, TupleConfigEnd<Config2> >(config) {}

template<class Config1, class Config2>
TupleConfig2<Config1, Config2>::TupleConfig2(const Config1& cfg1, const Config2& cfg2) :
			  TupleConfig<Config1, TupleConfigEnd<Config2> >(
					  cfg1, TupleConfigEnd<Config2>(cfg2)) {}

template<class Config1, class Config2, class Config3>
TupleConfig3<Config1, Config2, Config3>::TupleConfig3(const TupleConfig3<Config1, Config2, Config3>& config) :
		  TupleConfig<Config1, TupleConfig<Config2, TupleConfigEnd<Config3> > >(config) {}

template<class Config1, class Config2, class Config3>
TupleConfig3<Config1, Config2, Config3>::TupleConfig3(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3) :
			  TupleConfig<Config1, TupleConfig<Config2, TupleConfigEnd<Config3> > >(
					  cfg1, TupleConfig<Config2, TupleConfigEnd<Config3> >(
							  cfg2, TupleConfigEnd<Config3>(cfg3))) {}

template<class Config1, class Config2, class Config3, class Config4>
TupleConfig4<Config1, Config2, Config3, Config4>::TupleConfig4(const TupleConfig4<Config1, Config2, Config3, Config4>& config) :
	TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfigEnd<Config4> > > >(config) {}

template<class Config1, class Config2, class Config3, class Config4>
TupleConfig4<Config1, Config2, Config3, Config4>::TupleConfig4(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,const Config4& cfg4) :
		  TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfigEnd<Config4> > > >(
				  cfg1, TupleConfig<Config2, TupleConfig<Config3, TupleConfigEnd<Config4> > >(
						  cfg2, TupleConfig<Config3, TupleConfigEnd<Config4> >(
								  cfg3, TupleConfigEnd<Config4>(cfg4)))) {}

template<class Config1, class Config2, class Config3, class Config4, class Config5>
TupleConfig5<Config1, Config2, Config3, Config4, Config5>::TupleConfig5(const TupleConfig5<Config1, Config2, Config3, Config4, Config5>& config) :
		  TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfigEnd<Config5> > > > >(config) {}

template<class Config1, class Config2, class Config3, class Config4, class Config5>
TupleConfig5<Config1, Config2, Config3, Config4, Config5>::TupleConfig5(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
				   const Config4& cfg4, const Config5& cfg5) :
						   TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfigEnd<Config5> > > > >(
								   cfg1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfigEnd<Config5> > > >(
										   cfg2, TupleConfig<Config3, TupleConfig<Config4, TupleConfigEnd<Config5> > >(
												   cfg3, TupleConfig<Config4, TupleConfigEnd<Config5> >(
														   cfg4, TupleConfigEnd<Config5>(cfg5))))) {}

template<class Config1, class Config2, class Config3, class Config4, class Config5, class Config6>
TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6>::TupleConfig6(const TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6>& config) :
			  TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfig<Config5, TupleConfigEnd<Config6> > > > > >(config) {}

template<class Config1, class Config2, class Config3, class Config4, class Config5, class Config6>
TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6>::TupleConfig6(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
				   const Config4& cfg4, const Config5& cfg5, const Config6& cfg6) :
						   TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfig<Config5, TupleConfigEnd<Config6> > > > > >(
								   cfg1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfig<Config5, TupleConfigEnd<Config6> > > > >(
										   cfg2, TupleConfig<Config3, TupleConfig<Config4, TupleConfig<Config5, TupleConfigEnd<Config6> > > >(
												   cfg3, TupleConfig<Config4, TupleConfig<Config5, TupleConfigEnd<Config6> > >(
														   cfg4, TupleConfig<Config5, TupleConfigEnd<Config6> >(
																   cfg5, TupleConfigEnd<Config6>(cfg6)))))) {}

}

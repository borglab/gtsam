/**
 * @file TupleValues-inl.h
 * @author Richard Roberts
 * @author Manohar Paluri
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/LieValues-inl.h>
#include <gtsam/nonlinear/TupleValues.h>

// TupleValues instantiations for N = 1-6
#define INSTANTIATE_TUPLE_CONFIG1(Config1) \
		template class TupleValues1<Config1>;

#define INSTANTIATE_TUPLE_CONFIG2(Config1, Config2) \
		template class TupleValues2<Config1, Config2>;

#define INSTANTIATE_TUPLE_CONFIG3(Config1, Config2, Config3) \
		template class TupleValues3<Config1, Config2, Config3>;

#define INSTANTIATE_TUPLE_CONFIG4(Config1, Config2, Config3, Config4) \
		template class TupleValues4<Config1, Config2, Config3, Config4>;

#define INSTANTIATE_TUPLE_CONFIG5(Config1, Config2, Config3, Config4, Config5) \
		template class TupleValues5<Config1, Config2, Config3, Config4, Config5>;

#define INSTANTIATE_TUPLE_CONFIG6(Config1, Config2, Config3, Config4, Config5, Config6) \
		template class TupleValues6<Config1, Config2, Config3, Config4, Config5, Config6>;


namespace gtsam {

/* ************************************************************************* */
/** TupleValuesN Implementations */
/* ************************************************************************* */

/* ************************************************************************* */
/** TupleValues 1 */
template<class Config1>
TupleValues1<Config1>::TupleValues1(const TupleValues1<Config1>& config) :
		  TupleValuesEnd<Config1> (config) {}

template<class Config1>
TupleValues1<Config1>::TupleValues1(const Config1& cfg1) :
			  TupleValuesEnd<Config1> (cfg1) {}

template<class Config1>
TupleValues1<Config1>::TupleValues1(const TupleValuesEnd<Config1>& config) :
	TupleValuesEnd<Config1>(config) {}

/* ************************************************************************* */
/** TupleValues 2 */
template<class Config1, class Config2>
TupleValues2<Config1, Config2>::TupleValues2(const TupleValues2<Config1, Config2>& config) :
		  TupleValues<Config1, TupleValuesEnd<Config2> >(config) {}

template<class Config1, class Config2>
TupleValues2<Config1, Config2>::TupleValues2(const Config1& cfg1, const Config2& cfg2) :
			  TupleValues<Config1, TupleValuesEnd<Config2> >(
					  cfg1, TupleValuesEnd<Config2>(cfg2)) {}

template<class Config1, class Config2>
TupleValues2<Config1, Config2>::TupleValues2(const TupleValues<Config1, TupleValuesEnd<Config2> >& config) :
	TupleValues<Config1, TupleValuesEnd<Config2> >(config) {}

/* ************************************************************************* */
/** TupleValues 3 */
template<class Config1, class Config2, class Config3>
TupleValues3<Config1, Config2, Config3>::TupleValues3(
		const TupleValues3<Config1, Config2, Config3>& config) :
		  TupleValues<Config1, TupleValues<Config2, TupleValuesEnd<Config3> > >(config) {}

template<class Config1, class Config2, class Config3>
TupleValues3<Config1, Config2, Config3>::TupleValues3(
		const Config1& cfg1, const Config2& cfg2, const Config3& cfg3) :
			  TupleValues<Config1, TupleValues<Config2, TupleValuesEnd<Config3> > >(
					  cfg1, TupleValues<Config2, TupleValuesEnd<Config3> >(
							  cfg2, TupleValuesEnd<Config3>(cfg3))) {}

template<class Config1, class Config2, class Config3>
TupleValues3<Config1, Config2, Config3>::TupleValues3(
		const TupleValues<Config1, TupleValues<Config2, TupleValuesEnd<Config3> > >& config) :
		  TupleValues<Config1, TupleValues<Config2, TupleValuesEnd<Config3> > >(config) {}

/* ************************************************************************* */
/** TupleValues 4 */
template<class Config1, class Config2, class Config3, class Config4>
TupleValues4<Config1, Config2, Config3, Config4>::TupleValues4(
		const TupleValues4<Config1, Config2, Config3, Config4>& config) :
	TupleValues<Config1, TupleValues<Config2,
		TupleValues<Config3, TupleValuesEnd<Config4> > > >(config) {}

template<class Config1, class Config2, class Config3, class Config4>
TupleValues4<Config1, Config2, Config3, Config4>::TupleValues4(
		const Config1& cfg1, const Config2& cfg2,
		const Config3& cfg3,const Config4& cfg4) :
		  TupleValues<Config1, TupleValues<Config2,
			  TupleValues<Config3, TupleValuesEnd<Config4> > > >(
				  cfg1, TupleValues<Config2, TupleValues<Config3, TupleValuesEnd<Config4> > >(
						  cfg2, TupleValues<Config3, TupleValuesEnd<Config4> >(
								  cfg3, TupleValuesEnd<Config4>(cfg4)))) {}

template<class Config1, class Config2, class Config3, class Config4>
TupleValues4<Config1, Config2, Config3, Config4>::TupleValues4(
		const TupleValues<Config1, TupleValues<Config2,
				TupleValues<Config3, TupleValuesEnd<Config4> > > >& config) :
	TupleValues<Config1, TupleValues<Config2,TupleValues<Config3,
		TupleValuesEnd<Config4> > > >(config) {}

/* ************************************************************************* */
/** TupleValues 5 */
template<class Config1, class Config2, class Config3, class Config4, class Config5>
TupleValues5<Config1, Config2, Config3, Config4, Config5>::TupleValues5(
		const TupleValues5<Config1, Config2, Config3, Config4, Config5>& config) :
		  TupleValues<Config1, TupleValues<Config2, TupleValues<Config3,
			  TupleValues<Config4, TupleValuesEnd<Config5> > > > >(config) {}

template<class Config1, class Config2, class Config3, class Config4, class Config5>
TupleValues5<Config1, Config2, Config3, Config4, Config5>::TupleValues5(
		const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
				   const Config4& cfg4, const Config5& cfg5) :
						   TupleValues<Config1, TupleValues<Config2,
						   TupleValues<Config3, TupleValues<Config4,
						   TupleValuesEnd<Config5> > > > >(
								   cfg1, TupleValues<Config2, TupleValues<Config3,
										 TupleValues<Config4, TupleValuesEnd<Config5> > > >(
										   cfg2, TupleValues<Config3, TupleValues<Config4, TupleValuesEnd<Config5> > >(
												   cfg3, TupleValues<Config4, TupleValuesEnd<Config5> >(
														   cfg4, TupleValuesEnd<Config5>(cfg5))))) {}

template<class Config1, class Config2, class Config3, class Config4, class Config5>
TupleValues5<Config1, Config2, Config3, Config4, Config5>::TupleValues5(
		const TupleValues<Config1, TupleValues<Config2, TupleValues<Config3,
			  TupleValues<Config4, TupleValuesEnd<Config5> > > > >& config) :
	TupleValues<Config1, TupleValues<Config2, TupleValues<Config3,
	TupleValues<Config4, TupleValuesEnd<Config5> > > > >(config) {}

/* ************************************************************************* */
/** TupleValues 6 */
template<class Config1, class Config2, class Config3,
		 class Config4, class Config5, class Config6>
TupleValues6<Config1, Config2, Config3, Config4, Config5, Config6>::TupleValues6(
		const TupleValues6<Config1, Config2, Config3,
						   Config4, Config5, Config6>& config) :
			  TupleValues<Config1, TupleValues<Config2, TupleValues<Config3,
			  TupleValues<Config4, TupleValues<Config5,
			  TupleValuesEnd<Config6> > > > > >(config) {}

template<class Config1, class Config2, class Config3,
		 class Config4, class Config5, class Config6>
TupleValues6<Config1, Config2, Config3, Config4, Config5, Config6>::TupleValues6(
		const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
		const Config4& cfg4, const Config5& cfg5, const Config6& cfg6) :
		TupleValues<Config1, TupleValues<Config2, TupleValues<Config3,
		TupleValues<Config4, TupleValues<Config5, TupleValuesEnd<Config6> > > > > >(
				cfg1, TupleValues<Config2, TupleValues<Config3, TupleValues<Config4,
				      TupleValues<Config5, TupleValuesEnd<Config6> > > > >(
						cfg2, TupleValues<Config3, TupleValues<Config4, TupleValues<Config5,
							  TupleValuesEnd<Config6> > > >(
								cfg3, TupleValues<Config4, TupleValues<Config5,
								      TupleValuesEnd<Config6> > >(
										cfg4, TupleValues<Config5, TupleValuesEnd<Config6> >(
												cfg5, TupleValuesEnd<Config6>(cfg6)))))) {}

template<class Config1, class Config2, class Config3,
	     class Config4, class Config5, class Config6>
TupleValues6<Config1, Config2, Config3, Config4, Config5, Config6>::TupleValues6(
		const TupleValues<Config1, TupleValues<Config2, TupleValues<Config3,
		      TupleValues<Config4, TupleValues<Config5,
		      TupleValuesEnd<Config6> > > > > >& config) :
	TupleValues<Config1, TupleValues<Config2, TupleValues<Config3,
	TupleValues<Config4, TupleValues<Config5,
	TupleValuesEnd<Config6> > > > > >(config) {}

}

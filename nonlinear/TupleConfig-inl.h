/**
 * @file TupleConfig-inl.h
 * @author Richard Roberts
 * @author Manohar Paluri
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/LieConfig-inl.h>
#include <gtsam/nonlinear/TupleConfig.h>

// TupleConfig instantiations for N = 1-6
#define INSTANTIATE_TUPLE_CONFIG1(Config1) \
		template class TupleConfig1<Config1>;

#define INSTANTIATE_TUPLE_CONFIG2(Config1, Config2) \
		template class TupleConfig2<Config1, Config2>;

#define INSTANTIATE_TUPLE_CONFIG3(Config1, Config2, Config3) \
		template class TupleConfig3<Config1, Config2, Config3>;

#define INSTANTIATE_TUPLE_CONFIG4(Config1, Config2, Config3, Config4) \
		template class TupleConfig4<Config1, Config2, Config3, Config4>;

#define INSTANTIATE_TUPLE_CONFIG5(Config1, Config2, Config3, Config4, Config5) \
		template class TupleConfig5<Config1, Config2, Config3, Config4, Config5>;

#define INSTANTIATE_TUPLE_CONFIG6(Config1, Config2, Config3, Config4, Config5, Config6) \
		template class TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6>;


namespace gtsam {

/* ************************************************************************* */
/** TupleConfigN Implementations */
/* ************************************************************************* */

/* ************************************************************************* */
/** TupleConfig 1 */
template<class Config1>
TupleConfig1<Config1>::TupleConfig1(const TupleConfig1<Config1>& config) :
		  TupleConfigEnd<Config1> (config) {}

template<class Config1>
TupleConfig1<Config1>::TupleConfig1(const Config1& cfg1) :
			  TupleConfigEnd<Config1> (cfg1) {}

template<class Config1>
TupleConfig1<Config1>::TupleConfig1(const TupleConfigEnd<Config1>& config) :
	TupleConfigEnd<Config1>(config) {}

/* ************************************************************************* */
/** TupleConfig 2 */
template<class Config1, class Config2>
TupleConfig2<Config1, Config2>::TupleConfig2(const TupleConfig2<Config1, Config2>& config) :
		  TupleConfig<Config1, TupleConfigEnd<Config2> >(config) {}

template<class Config1, class Config2>
TupleConfig2<Config1, Config2>::TupleConfig2(const Config1& cfg1, const Config2& cfg2) :
			  TupleConfig<Config1, TupleConfigEnd<Config2> >(
					  cfg1, TupleConfigEnd<Config2>(cfg2)) {}

template<class Config1, class Config2>
TupleConfig2<Config1, Config2>::TupleConfig2(const TupleConfig<Config1, TupleConfigEnd<Config2> >& config) :
	TupleConfig<Config1, TupleConfigEnd<Config2> >(config) {}

/* ************************************************************************* */
/** TupleConfig 3 */
template<class Config1, class Config2, class Config3>
TupleConfig3<Config1, Config2, Config3>::TupleConfig3(
		const TupleConfig3<Config1, Config2, Config3>& config) :
		  TupleConfig<Config1, TupleConfig<Config2, TupleConfigEnd<Config3> > >(config) {}

template<class Config1, class Config2, class Config3>
TupleConfig3<Config1, Config2, Config3>::TupleConfig3(
		const Config1& cfg1, const Config2& cfg2, const Config3& cfg3) :
			  TupleConfig<Config1, TupleConfig<Config2, TupleConfigEnd<Config3> > >(
					  cfg1, TupleConfig<Config2, TupleConfigEnd<Config3> >(
							  cfg2, TupleConfigEnd<Config3>(cfg3))) {}

template<class Config1, class Config2, class Config3>
TupleConfig3<Config1, Config2, Config3>::TupleConfig3(
		const TupleConfig<Config1, TupleConfig<Config2, TupleConfigEnd<Config3> > >& config) :
		  TupleConfig<Config1, TupleConfig<Config2, TupleConfigEnd<Config3> > >(config) {}

/* ************************************************************************* */
/** TupleConfig 4 */
template<class Config1, class Config2, class Config3, class Config4>
TupleConfig4<Config1, Config2, Config3, Config4>::TupleConfig4(
		const TupleConfig4<Config1, Config2, Config3, Config4>& config) :
	TupleConfig<Config1, TupleConfig<Config2,
		TupleConfig<Config3, TupleConfigEnd<Config4> > > >(config) {}

template<class Config1, class Config2, class Config3, class Config4>
TupleConfig4<Config1, Config2, Config3, Config4>::TupleConfig4(
		const Config1& cfg1, const Config2& cfg2,
		const Config3& cfg3,const Config4& cfg4) :
		  TupleConfig<Config1, TupleConfig<Config2,
			  TupleConfig<Config3, TupleConfigEnd<Config4> > > >(
				  cfg1, TupleConfig<Config2, TupleConfig<Config3, TupleConfigEnd<Config4> > >(
						  cfg2, TupleConfig<Config3, TupleConfigEnd<Config4> >(
								  cfg3, TupleConfigEnd<Config4>(cfg4)))) {}

template<class Config1, class Config2, class Config3, class Config4>
TupleConfig4<Config1, Config2, Config3, Config4>::TupleConfig4(
		const TupleConfig<Config1, TupleConfig<Config2,
				TupleConfig<Config3, TupleConfigEnd<Config4> > > >& config) :
	TupleConfig<Config1, TupleConfig<Config2,TupleConfig<Config3,
		TupleConfigEnd<Config4> > > >(config) {}

/* ************************************************************************* */
/** TupleConfig 5 */
template<class Config1, class Config2, class Config3, class Config4, class Config5>
TupleConfig5<Config1, Config2, Config3, Config4, Config5>::TupleConfig5(
		const TupleConfig5<Config1, Config2, Config3, Config4, Config5>& config) :
		  TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3,
			  TupleConfig<Config4, TupleConfigEnd<Config5> > > > >(config) {}

template<class Config1, class Config2, class Config3, class Config4, class Config5>
TupleConfig5<Config1, Config2, Config3, Config4, Config5>::TupleConfig5(
		const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
				   const Config4& cfg4, const Config5& cfg5) :
						   TupleConfig<Config1, TupleConfig<Config2,
						   TupleConfig<Config3, TupleConfig<Config4,
						   TupleConfigEnd<Config5> > > > >(
								   cfg1, TupleConfig<Config2, TupleConfig<Config3,
										 TupleConfig<Config4, TupleConfigEnd<Config5> > > >(
										   cfg2, TupleConfig<Config3, TupleConfig<Config4, TupleConfigEnd<Config5> > >(
												   cfg3, TupleConfig<Config4, TupleConfigEnd<Config5> >(
														   cfg4, TupleConfigEnd<Config5>(cfg5))))) {}

template<class Config1, class Config2, class Config3, class Config4, class Config5>
TupleConfig5<Config1, Config2, Config3, Config4, Config5>::TupleConfig5(
		const TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3,
			  TupleConfig<Config4, TupleConfigEnd<Config5> > > > >& config) :
	TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3,
	TupleConfig<Config4, TupleConfigEnd<Config5> > > > >(config) {}

/* ************************************************************************* */
/** TupleConfig 6 */
template<class Config1, class Config2, class Config3,
		 class Config4, class Config5, class Config6>
TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6>::TupleConfig6(
		const TupleConfig6<Config1, Config2, Config3,
						   Config4, Config5, Config6>& config) :
			  TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3,
			  TupleConfig<Config4, TupleConfig<Config5,
			  TupleConfigEnd<Config6> > > > > >(config) {}

template<class Config1, class Config2, class Config3,
		 class Config4, class Config5, class Config6>
TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6>::TupleConfig6(
		const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
		const Config4& cfg4, const Config5& cfg5, const Config6& cfg6) :
		TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3,
		TupleConfig<Config4, TupleConfig<Config5, TupleConfigEnd<Config6> > > > > >(
				cfg1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4,
				      TupleConfig<Config5, TupleConfigEnd<Config6> > > > >(
						cfg2, TupleConfig<Config3, TupleConfig<Config4, TupleConfig<Config5,
							  TupleConfigEnd<Config6> > > >(
								cfg3, TupleConfig<Config4, TupleConfig<Config5,
								      TupleConfigEnd<Config6> > >(
										cfg4, TupleConfig<Config5, TupleConfigEnd<Config6> >(
												cfg5, TupleConfigEnd<Config6>(cfg6)))))) {}

template<class Config1, class Config2, class Config3,
	     class Config4, class Config5, class Config6>
TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6>::TupleConfig6(
		const TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3,
		      TupleConfig<Config4, TupleConfig<Config5,
		      TupleConfigEnd<Config6> > > > > >& config) :
	TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3,
	TupleConfig<Config4, TupleConfig<Config5,
	TupleConfigEnd<Config6> > > > > >(config) {}

}

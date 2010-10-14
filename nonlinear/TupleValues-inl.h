/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
#define INSTANTIATE_TUPLE_CONFIG1(Values1) \
		template class TupleValues1<Values1>;

#define INSTANTIATE_TUPLE_CONFIG2(Values1, Values2) \
		template class TupleValues2<Values1, Values2>;

#define INSTANTIATE_TUPLE_CONFIG3(Values1, Values2, Values3) \
		template class TupleValues3<Values1, Values2, Values3>;

#define INSTANTIATE_TUPLE_CONFIG4(Values1, Values2, Values3, Values4) \
		template class TupleValues4<Values1, Values2, Values3, Values4>;

#define INSTANTIATE_TUPLE_CONFIG5(Values1, Values2, Values3, Values4, Values5) \
		template class TupleValues5<Values1, Values2, Values3, Values4, Values5>;

#define INSTANTIATE_TUPLE_CONFIG6(Values1, Values2, Values3, Values4, Values5, Values6) \
		template class TupleValues6<Values1, Values2, Values3, Values4, Values5, Values6>;


namespace gtsam {

/* ************************************************************************* */
/** TupleValuesN Implementations */
/* ************************************************************************* */

/* ************************************************************************* */
/** TupleValues 1 */
template<class Values1>
TupleValues1<Values1>::TupleValues1(const TupleValues1<Values1>& config) :
		  TupleValuesEnd<Values1> (config) {}

template<class Values1>
TupleValues1<Values1>::TupleValues1(const Values1& cfg1) :
			  TupleValuesEnd<Values1> (cfg1) {}

template<class Values1>
TupleValues1<Values1>::TupleValues1(const TupleValuesEnd<Values1>& config) :
	TupleValuesEnd<Values1>(config) {}

/* ************************************************************************* */
/** TupleValues 2 */
template<class Values1, class Values2>
TupleValues2<Values1, Values2>::TupleValues2(const TupleValues2<Values1, Values2>& config) :
		  TupleValues<Values1, TupleValuesEnd<Values2> >(config) {}

template<class Values1, class Values2>
TupleValues2<Values1, Values2>::TupleValues2(const Values1& cfg1, const Values2& cfg2) :
			  TupleValues<Values1, TupleValuesEnd<Values2> >(
					  cfg1, TupleValuesEnd<Values2>(cfg2)) {}

template<class Values1, class Values2>
TupleValues2<Values1, Values2>::TupleValues2(const TupleValues<Values1, TupleValuesEnd<Values2> >& config) :
	TupleValues<Values1, TupleValuesEnd<Values2> >(config) {}

/* ************************************************************************* */
/** TupleValues 3 */
template<class Values1, class Values2, class Values3>
TupleValues3<Values1, Values2, Values3>::TupleValues3(
		const TupleValues3<Values1, Values2, Values3>& config) :
		  TupleValues<Values1, TupleValues<Values2, TupleValuesEnd<Values3> > >(config) {}

template<class Values1, class Values2, class Values3>
TupleValues3<Values1, Values2, Values3>::TupleValues3(
		const Values1& cfg1, const Values2& cfg2, const Values3& cfg3) :
			  TupleValues<Values1, TupleValues<Values2, TupleValuesEnd<Values3> > >(
					  cfg1, TupleValues<Values2, TupleValuesEnd<Values3> >(
							  cfg2, TupleValuesEnd<Values3>(cfg3))) {}

template<class Values1, class Values2, class Values3>
TupleValues3<Values1, Values2, Values3>::TupleValues3(
		const TupleValues<Values1, TupleValues<Values2, TupleValuesEnd<Values3> > >& config) :
		  TupleValues<Values1, TupleValues<Values2, TupleValuesEnd<Values3> > >(config) {}

/* ************************************************************************* */
/** TupleValues 4 */
template<class Values1, class Values2, class Values3, class Values4>
TupleValues4<Values1, Values2, Values3, Values4>::TupleValues4(
		const TupleValues4<Values1, Values2, Values3, Values4>& config) :
	TupleValues<Values1, TupleValues<Values2,
		TupleValues<Values3, TupleValuesEnd<Values4> > > >(config) {}

template<class Values1, class Values2, class Values3, class Values4>
TupleValues4<Values1, Values2, Values3, Values4>::TupleValues4(
		const Values1& cfg1, const Values2& cfg2,
		const Values3& cfg3,const Values4& cfg4) :
		  TupleValues<Values1, TupleValues<Values2,
			  TupleValues<Values3, TupleValuesEnd<Values4> > > >(
				  cfg1, TupleValues<Values2, TupleValues<Values3, TupleValuesEnd<Values4> > >(
						  cfg2, TupleValues<Values3, TupleValuesEnd<Values4> >(
								  cfg3, TupleValuesEnd<Values4>(cfg4)))) {}

template<class Values1, class Values2, class Values3, class Values4>
TupleValues4<Values1, Values2, Values3, Values4>::TupleValues4(
		const TupleValues<Values1, TupleValues<Values2,
				TupleValues<Values3, TupleValuesEnd<Values4> > > >& config) :
	TupleValues<Values1, TupleValues<Values2,TupleValues<Values3,
		TupleValuesEnd<Values4> > > >(config) {}

/* ************************************************************************* */
/** TupleValues 5 */
template<class Values1, class Values2, class Values3, class Values4, class Values5>
TupleValues5<Values1, Values2, Values3, Values4, Values5>::TupleValues5(
		const TupleValues5<Values1, Values2, Values3, Values4, Values5>& config) :
		  TupleValues<Values1, TupleValues<Values2, TupleValues<Values3,
			  TupleValues<Values4, TupleValuesEnd<Values5> > > > >(config) {}

template<class Values1, class Values2, class Values3, class Values4, class Values5>
TupleValues5<Values1, Values2, Values3, Values4, Values5>::TupleValues5(
		const Values1& cfg1, const Values2& cfg2, const Values3& cfg3,
				   const Values4& cfg4, const Values5& cfg5) :
						   TupleValues<Values1, TupleValues<Values2,
						   TupleValues<Values3, TupleValues<Values4,
						   TupleValuesEnd<Values5> > > > >(
								   cfg1, TupleValues<Values2, TupleValues<Values3,
										 TupleValues<Values4, TupleValuesEnd<Values5> > > >(
										   cfg2, TupleValues<Values3, TupleValues<Values4, TupleValuesEnd<Values5> > >(
												   cfg3, TupleValues<Values4, TupleValuesEnd<Values5> >(
														   cfg4, TupleValuesEnd<Values5>(cfg5))))) {}

template<class Values1, class Values2, class Values3, class Values4, class Values5>
TupleValues5<Values1, Values2, Values3, Values4, Values5>::TupleValues5(
		const TupleValues<Values1, TupleValues<Values2, TupleValues<Values3,
			  TupleValues<Values4, TupleValuesEnd<Values5> > > > >& config) :
	TupleValues<Values1, TupleValues<Values2, TupleValues<Values3,
	TupleValues<Values4, TupleValuesEnd<Values5> > > > >(config) {}

/* ************************************************************************* */
/** TupleValues 6 */
template<class Values1, class Values2, class Values3,
		 class Values4, class Values5, class Values6>
TupleValues6<Values1, Values2, Values3, Values4, Values5, Values6>::TupleValues6(
		const TupleValues6<Values1, Values2, Values3,
						   Values4, Values5, Values6>& config) :
			  TupleValues<Values1, TupleValues<Values2, TupleValues<Values3,
			  TupleValues<Values4, TupleValues<Values5,
			  TupleValuesEnd<Values6> > > > > >(config) {}

template<class Values1, class Values2, class Values3,
		 class Values4, class Values5, class Values6>
TupleValues6<Values1, Values2, Values3, Values4, Values5, Values6>::TupleValues6(
		const Values1& cfg1, const Values2& cfg2, const Values3& cfg3,
		const Values4& cfg4, const Values5& cfg5, const Values6& cfg6) :
		TupleValues<Values1, TupleValues<Values2, TupleValues<Values3,
		TupleValues<Values4, TupleValues<Values5, TupleValuesEnd<Values6> > > > > >(
				cfg1, TupleValues<Values2, TupleValues<Values3, TupleValues<Values4,
				      TupleValues<Values5, TupleValuesEnd<Values6> > > > >(
						cfg2, TupleValues<Values3, TupleValues<Values4, TupleValues<Values5,
							  TupleValuesEnd<Values6> > > >(
								cfg3, TupleValues<Values4, TupleValues<Values5,
								      TupleValuesEnd<Values6> > >(
										cfg4, TupleValues<Values5, TupleValuesEnd<Values6> >(
												cfg5, TupleValuesEnd<Values6>(cfg6)))))) {}

template<class Values1, class Values2, class Values3,
	     class Values4, class Values5, class Values6>
TupleValues6<Values1, Values2, Values3, Values4, Values5, Values6>::TupleValues6(
		const TupleValues<Values1, TupleValues<Values2, TupleValues<Values3,
		      TupleValues<Values4, TupleValues<Values5,
		      TupleValuesEnd<Values6> > > > > >& config) :
	TupleValues<Values1, TupleValues<Values2, TupleValues<Values3,
	TupleValues<Values4, TupleValues<Values5,
	TupleValuesEnd<Values6> > > > > >(config) {}

}

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
template<class VALUES1>
TupleValues1<VALUES1>::TupleValues1(const TupleValues1<VALUES1>& config) :
		  TupleValuesEnd<VALUES1> (config) {}

template<class VALUES1>
TupleValues1<VALUES1>::TupleValues1(const VALUES1& cfg1) :
			  TupleValuesEnd<VALUES1> (cfg1) {}

template<class VALUES1>
TupleValues1<VALUES1>::TupleValues1(const TupleValuesEnd<VALUES1>& config) :
	TupleValuesEnd<VALUES1>(config) {}

/* ************************************************************************* */
/** TupleValues 2 */
template<class VALUES1, class VALUES2>
TupleValues2<VALUES1, VALUES2>::TupleValues2(const TupleValues2<VALUES1, VALUES2>& config) :
		  TupleValues<VALUES1, TupleValuesEnd<VALUES2> >(config) {}

template<class VALUES1, class VALUES2>
TupleValues2<VALUES1, VALUES2>::TupleValues2(const VALUES1& cfg1, const VALUES2& cfg2) :
			  TupleValues<VALUES1, TupleValuesEnd<VALUES2> >(
					  cfg1, TupleValuesEnd<VALUES2>(cfg2)) {}

template<class VALUES1, class VALUES2>
TupleValues2<VALUES1, VALUES2>::TupleValues2(const TupleValues<VALUES1, TupleValuesEnd<VALUES2> >& config) :
	TupleValues<VALUES1, TupleValuesEnd<VALUES2> >(config) {}

/* ************************************************************************* */
/** TupleValues 3 */
template<class VALUES1, class VALUES2, class VALUES3>
TupleValues3<VALUES1, VALUES2, VALUES3>::TupleValues3(
		const TupleValues3<VALUES1, VALUES2, VALUES3>& config) :
		  TupleValues<VALUES1, TupleValues<VALUES2, TupleValuesEnd<VALUES3> > >(config) {}

template<class VALUES1, class VALUES2, class VALUES3>
TupleValues3<VALUES1, VALUES2, VALUES3>::TupleValues3(
		const VALUES1& cfg1, const VALUES2& cfg2, const VALUES3& cfg3) :
			  TupleValues<VALUES1, TupleValues<VALUES2, TupleValuesEnd<VALUES3> > >(
					  cfg1, TupleValues<VALUES2, TupleValuesEnd<VALUES3> >(
							  cfg2, TupleValuesEnd<VALUES3>(cfg3))) {}

template<class VALUES1, class VALUES2, class VALUES3>
TupleValues3<VALUES1, VALUES2, VALUES3>::TupleValues3(
		const TupleValues<VALUES1, TupleValues<VALUES2, TupleValuesEnd<VALUES3> > >& config) :
		  TupleValues<VALUES1, TupleValues<VALUES2, TupleValuesEnd<VALUES3> > >(config) {}

/* ************************************************************************* */
/** TupleValues 4 */
template<class VALUES1, class VALUES2, class VALUES3, class VALUES4>
TupleValues4<VALUES1, VALUES2, VALUES3, VALUES4>::TupleValues4(
		const TupleValues4<VALUES1, VALUES2, VALUES3, VALUES4>& config) :
	TupleValues<VALUES1, TupleValues<VALUES2,
		TupleValues<VALUES3, TupleValuesEnd<VALUES4> > > >(config) {}

template<class VALUES1, class VALUES2, class VALUES3, class VALUES4>
TupleValues4<VALUES1, VALUES2, VALUES3, VALUES4>::TupleValues4(
		const VALUES1& cfg1, const VALUES2& cfg2,
		const VALUES3& cfg3,const VALUES4& cfg4) :
		  TupleValues<VALUES1, TupleValues<VALUES2,
			  TupleValues<VALUES3, TupleValuesEnd<VALUES4> > > >(
				  cfg1, TupleValues<VALUES2, TupleValues<VALUES3, TupleValuesEnd<VALUES4> > >(
						  cfg2, TupleValues<VALUES3, TupleValuesEnd<VALUES4> >(
								  cfg3, TupleValuesEnd<VALUES4>(cfg4)))) {}

template<class VALUES1, class VALUES2, class VALUES3, class VALUES4>
TupleValues4<VALUES1, VALUES2, VALUES3, VALUES4>::TupleValues4(
		const TupleValues<VALUES1, TupleValues<VALUES2,
				TupleValues<VALUES3, TupleValuesEnd<VALUES4> > > >& config) :
	TupleValues<VALUES1, TupleValues<VALUES2,TupleValues<VALUES3,
		TupleValuesEnd<VALUES4> > > >(config) {}

/* ************************************************************************* */
/** TupleValues 5 */
template<class VALUES1, class VALUES2, class VALUES3, class VALUES4, class VALUES5>
TupleValues5<VALUES1, VALUES2, VALUES3, VALUES4, VALUES5>::TupleValues5(
		const TupleValues5<VALUES1, VALUES2, VALUES3, VALUES4, VALUES5>& config) :
		  TupleValues<VALUES1, TupleValues<VALUES2, TupleValues<VALUES3,
			  TupleValues<VALUES4, TupleValuesEnd<VALUES5> > > > >(config) {}

template<class VALUES1, class VALUES2, class VALUES3, class VALUES4, class VALUES5>
TupleValues5<VALUES1, VALUES2, VALUES3, VALUES4, VALUES5>::TupleValues5(
		const VALUES1& cfg1, const VALUES2& cfg2, const VALUES3& cfg3,
				   const VALUES4& cfg4, const VALUES5& cfg5) :
						   TupleValues<VALUES1, TupleValues<VALUES2,
						   TupleValues<VALUES3, TupleValues<VALUES4,
						   TupleValuesEnd<VALUES5> > > > >(
								   cfg1, TupleValues<VALUES2, TupleValues<VALUES3,
										 TupleValues<VALUES4, TupleValuesEnd<VALUES5> > > >(
										   cfg2, TupleValues<VALUES3, TupleValues<VALUES4, TupleValuesEnd<VALUES5> > >(
												   cfg3, TupleValues<VALUES4, TupleValuesEnd<VALUES5> >(
														   cfg4, TupleValuesEnd<VALUES5>(cfg5))))) {}

template<class VALUES1, class VALUES2, class VALUES3, class VALUES4, class VALUES5>
TupleValues5<VALUES1, VALUES2, VALUES3, VALUES4, VALUES5>::TupleValues5(
		const TupleValues<VALUES1, TupleValues<VALUES2, TupleValues<VALUES3,
			  TupleValues<VALUES4, TupleValuesEnd<VALUES5> > > > >& config) :
	TupleValues<VALUES1, TupleValues<VALUES2, TupleValues<VALUES3,
	TupleValues<VALUES4, TupleValuesEnd<VALUES5> > > > >(config) {}

/* ************************************************************************* */
/** TupleValues 6 */
template<class VALUES1, class VALUES2, class VALUES3,
		 class VALUES4, class VALUES5, class VALUES6>
TupleValues6<VALUES1, VALUES2, VALUES3, VALUES4, VALUES5, VALUES6>::TupleValues6(
		const TupleValues6<VALUES1, VALUES2, VALUES3,
						   VALUES4, VALUES5, VALUES6>& config) :
			  TupleValues<VALUES1, TupleValues<VALUES2, TupleValues<VALUES3,
			  TupleValues<VALUES4, TupleValues<VALUES5,
			  TupleValuesEnd<VALUES6> > > > > >(config) {}

template<class VALUES1, class VALUES2, class VALUES3,
		 class VALUES4, class VALUES5, class VALUES6>
TupleValues6<VALUES1, VALUES2, VALUES3, VALUES4, VALUES5, VALUES6>::TupleValues6(
		const VALUES1& cfg1, const VALUES2& cfg2, const VALUES3& cfg3,
		const VALUES4& cfg4, const VALUES5& cfg5, const VALUES6& cfg6) :
		TupleValues<VALUES1, TupleValues<VALUES2, TupleValues<VALUES3,
		TupleValues<VALUES4, TupleValues<VALUES5, TupleValuesEnd<VALUES6> > > > > >(
				cfg1, TupleValues<VALUES2, TupleValues<VALUES3, TupleValues<VALUES4,
				      TupleValues<VALUES5, TupleValuesEnd<VALUES6> > > > >(
						cfg2, TupleValues<VALUES3, TupleValues<VALUES4, TupleValues<VALUES5,
							  TupleValuesEnd<VALUES6> > > >(
								cfg3, TupleValues<VALUES4, TupleValues<VALUES5,
								      TupleValuesEnd<VALUES6> > >(
										cfg4, TupleValues<VALUES5, TupleValuesEnd<VALUES6> >(
												cfg5, TupleValuesEnd<VALUES6>(cfg6)))))) {}

template<class VALUES1, class VALUES2, class VALUES3,
	     class VALUES4, class VALUES5, class VALUES6>
TupleValues6<VALUES1, VALUES2, VALUES3, VALUES4, VALUES5, VALUES6>::TupleValues6(
		const TupleValues<VALUES1, TupleValues<VALUES2, TupleValues<VALUES3,
		      TupleValues<VALUES4, TupleValues<VALUES5,
		      TupleValuesEnd<VALUES6> > > > > >& config) :
	TupleValues<VALUES1, TupleValues<VALUES2, TupleValues<VALUES3,
	TupleValues<VALUES4, TupleValues<VALUES5,
	TupleValuesEnd<VALUES6> > > > > >(config) {}

}

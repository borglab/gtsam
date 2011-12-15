/**
 * @file spirit_actors.h
 *
 * @brief Additional actors for the wrap parser
 *
 * @date Dec 8, 2011
 * @author Alex Cunningham
 */

#pragma once

#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_ref_actor.hpp>

namespace boost {
namespace spirit {

BOOST_SPIRIT_CLASSIC_NAMESPACE_BEGIN

///////////////////////////////////////////////////////////////////////////
//  Summary:
//  A semantic action policy that calls pop_back method.
//  (This doc uses convention available in actors.hpp)
//  Note that this action performs a "safe" pop, that checks the size
//  before popping to avoid segfaults
//
//  Actions (what it does):
//      ref.pop_back();
//
//  Policy name:
//      pop_action
//
//  Policy holder, corresponding helper method:
//      ref_actor, clear_a( ref );
//
//  () operators: both.
//
//  See also ref_actor for more details.
///////////////////////////////////////////////////////////////////////////
struct pop_action
{
	template<	typename T>
	void act(T& ref_) const
	{
		if (!ref_.empty())
		ref_.pop_back();
	}
};

///////////////////////////////////////////////////////////////////////////
// helper method that creates a and_assign_actor.
///////////////////////////////////////////////////////////////////////////
template<typename T>
inline ref_actor<T,pop_action> pop_a(T& ref_)
{
	return ref_actor<T,pop_action>(ref_);
}

///////////////////////////////////////////////////////////////////////////
//  Summary:
//
//  A semantic action policy that appends a set of values to the back of a
//  container.
//  (This doc uses convention available in actors.hpp)
//
//  Actions (what it does and what ref, value_ref must support):
//      ref.push_back( values );
//      ref.push_back( T::value_type(first,last) );
//      ref.push_back( value_ref );
//
//  Policy name:
//      append_action
//
//  Policy holder, corresponding helper method:
//      ref_value_actor, append_a( ref );
//      ref_const_ref_actor, append_a( ref, value_ref );
//
//  () operators: both
//
//  See also ref_value_actor and ref_const_ref_actor for more details.
///////////////////////////////////////////////////////////////////////////

struct append_action
{
	template<	typename T,	typename ValueT	>
	void act(T& ref_, ValueT const& value_) const
	{
		ref_.insert(ref_.begin(), value_.begin(), value_.end());
	}

	template<typename T,typename IteratorT>
	void act(
			T& ref_,
			IteratorT const& first_,
			IteratorT const& last_
	) const
	{
		ref_.insert(ref_.end(), first_, last_);
	}
};

template<typename T>
inline ref_value_actor<T,append_action>
append_a(T& ref_)
{
	return ref_value_actor<T,append_action>(ref_);
}

template<typename T,typename ValueT>
inline ref_const_ref_actor<T,ValueT,append_action>
append_a(
		T& ref_,
		ValueT const& value_
)
{
	return ref_const_ref_actor<T,ValueT,append_action>(ref_,value_);
}

BOOST_SPIRIT_CLASSIC_NAMESPACE_END

}
}


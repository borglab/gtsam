/**
 * @file pop_actor.h
 *
 * @brief An actor to pop a vector/container, based off of the clear_actor
 *
 * @date Dec 8, 2011
 * @author Alex Cunningham
 */

#pragma once

#include <boost/spirit/include/classic_ref_actor.hpp>

namespace boost { namespace spirit {

BOOST_SPIRIT_CLASSIC_NAMESPACE_BEGIN

    ///////////////////////////////////////////////////////////////////////////
    //  Summary:
    //  A semantic action policy that calls pop_back method.
    //  (This doc uses convention available in actors.hpp)
    //
    //  Actions (what it does):
    //      ref.pop_back();
    //
    //  Policy name:
    //      clear_action
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
        template<
            typename T
        >
        void act(T& ref_) const
        {
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

BOOST_SPIRIT_CLASSIC_NAMESPACE_END

}}


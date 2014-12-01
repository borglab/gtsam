/**
 * @file spirit_actors.h
 *
 * @brief Additional utilities and actors for the wrap parser
 *
 * @date Dec 8, 2011
 * @author Alex Cunningham
 * @author Frank Dellaert
 */

#pragma once

#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_push_back_actor.hpp>
#include <boost/spirit/include/classic_clear_actor.hpp>
#include <boost/spirit/include/classic_assign_actor.hpp>
#include <boost/spirit/include/classic_confix.hpp>

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
  template< typename T>
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
  template< typename T, typename ValueT >
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

namespace wrap {

namespace classic = BOOST_SPIRIT_CLASSIC_NS;

/// Some basic rules used by all parsers
template<typename ScannerT>
struct BasicRules {

  classic::rule<ScannerT> comments_p, basisType_p, eigenType_p, keywords_p,
      stlType_p, name_p, className_p, namespace_p;

  BasicRules() {

    using classic::comment_p;
    using classic::eol_p;
    using classic::str_p;
    using classic::alpha_p;
    using classic::lexeme_d;
    using classic::upper_p;
    using classic::lower_p;
    using classic::alnum_p;

    comments_p = comment_p("/*", "*/") | comment_p("//", eol_p);

    basisType_p = (str_p("string") | "bool" | "size_t" | "int" | "double"
        | "char" | "unsigned char");

    eigenType_p = (str_p("Vector") | "Matrix");

    keywords_p =
        (str_p("const") | "static" | "namespace" | "void" | basisType_p);

    stlType_p = (str_p("vector") | "list");

    name_p = lexeme_d[alpha_p >> *(alnum_p | '_')];

    className_p = (lexeme_d[upper_p >> *(alnum_p | '_')] - eigenType_p
        - keywords_p) | stlType_p;

    namespace_p = lexeme_d[lower_p >> *(alnum_p | '_')] - keywords_p;
  }
};
}


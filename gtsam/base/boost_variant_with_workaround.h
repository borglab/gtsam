/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    boost_variant_with_workaround.h
 * @brief   Includes boost/variant.hpp with a workaround for a variant/boost graph conflict, by defining the variant 'get' function in a new variant_workaround namespace.
 * @author  Richard Roberts
 */

#pragma once

#include <boost/variant.hpp>
#include <boost/version.hpp>

namespace variant_workaround {

  /* ************************************************************************* */
  // The following functions are pasted from boost/variant/get.hpp, but with
  // explicit argument types to avoid a conflict with 'get' in the boost graph
  // library that exists in boost < 1.47.
#if BOOST_VERSION >= 104700
  using boost::get;
#else
  template <typename U, BOOST_VARIANT_ENUM_PARAMS(typename T) >
  inline
  typename boost::add_pointer<U>::type
  get(
      boost::variant< BOOST_VARIANT_ENUM_PARAMS(T) >* operand
      BOOST_VARIANT_AUX_GET_EXPLICIT_TEMPLATE_TYPE(U)
  )
  {
    typedef typename boost::add_pointer<U>::type U_ptr;
    if (!operand) return static_cast<U_ptr>(0);

    boost::detail::variant::get_visitor<U> v;
    return operand->apply_visitor(v);
  }

  template <typename U, BOOST_VARIANT_ENUM_PARAMS(typename T) >
  inline
  typename boost::add_pointer<const U>::type
  get(
      const boost::variant< BOOST_VARIANT_ENUM_PARAMS(T) >* operand
      BOOST_VARIANT_AUX_GET_EXPLICIT_TEMPLATE_TYPE(U)
  )
  {
    typedef typename boost::add_pointer<const U>::type U_ptr;
    if (!operand) return static_cast<U_ptr>(0);

    boost::detail::variant::get_visitor<const U> v;
    return operand->apply_visitor(v);
  }

  template <typename U, BOOST_VARIANT_ENUM_PARAMS(typename T) >
  inline
  typename boost::add_reference<U>::type
  get(
      boost::variant< BOOST_VARIANT_ENUM_PARAMS(T) >& operand
      BOOST_VARIANT_AUX_GET_EXPLICIT_TEMPLATE_TYPE(U)
  )
  {
    typedef typename boost::add_pointer<U>::type U_ptr;
    U_ptr result = variant_workaround::get<U>(&operand);

    if (!result)
      throw boost::bad_get();
    return *result;
  }

  template <typename U, BOOST_VARIANT_ENUM_PARAMS(typename T) >
  inline
  typename boost::add_reference<const U>::type
  get(
      const boost::variant< BOOST_VARIANT_ENUM_PARAMS(T) >& operand
      BOOST_VARIANT_AUX_GET_EXPLICIT_TEMPLATE_TYPE(U)
  )
  {
    typedef typename boost::add_pointer<const U>::type U_ptr;
    U_ptr result = variant_workaround::get<const U>(&operand);

    if (!result)
      throw boost::bad_get();
    return *result;
  }
#endif

}

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    nonlinearExceptions.h
 * @brief   Exceptions that may be thrown by nonlinear optimization components
 * @author  Richard Roberts
 * @date    Aug 17, 2012
 */
#pragma once

#include <boost/lexical_cast.hpp>
#include <exception>

#include <gtsam/inference/Key.h>

namespace gtsam {

  /**
  Thrown when requesting to marginalize out variables from ISAM2 that are not
  leaves.  To make the variables you would like to marginalize be leaves, their
  ordering should be constrained using the constrainedKeys argument to
  ISAM2::update().
  */
  class MarginalizeNonleafException : public std::exception {
    Key key_;
    KeyFormatter formatter_;
    mutable std::string what_;
  public:
    MarginalizeNonleafException(Key key, KeyFormatter formatter = DefaultKeyFormatter) noexcept :
      key_(key), formatter_(formatter) {}
    virtual ~MarginalizeNonleafException() noexcept {}
    Key key() const { return key_; }
    const char* what() const noexcept override {
      if(what_.empty())
        what_ =
"\nRequested to marginalize out variable " + formatter_(key_) + ", but this variable\n\
is not a leaf.  To make the variables you would like to marginalize be leaves,\n\
their ordering should be constrained using the constrainedKeys argument to\n\
ISAM2::update().\n";
      return what_.c_str();
    }
  };

}

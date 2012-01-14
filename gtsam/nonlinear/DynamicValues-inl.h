/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DynamicValues.h
 * @author Richard Roberts
 *
 * @brief A non-templated config holding any types of Manifold-group elements
 *
 *  Detailed story:
 *  A values structure is a map from keys to values. It is used to specify the value of a bunch
 *  of variables in a factor graph. A Values is a values structure which can hold variables that
 *  are elements on manifolds, not just vectors. It then, as a whole, implements a aggregate type
 *  which is also a manifold element, and hence supports operations dim, retract, and localCoordinates.
 */

#include <gtsam/nonlinear/DynamicValues.h> // Only so Eclipse finds class definition

namespace gtsam {

  /* ************************************************************************* */
  const char* DynamicValuesIncorrectType::what() const throw() {
    if(message_.empty())
      message_ =
          "Attempting to retrieve value with key \"" + (std::string)key_ + "\", type stored in DynamicValues is " +
          std::string(storedTypeId_.name()) + " but requested type was " + std::string(requestedTypeId_.name());
    return message_.c_str();
  }

}

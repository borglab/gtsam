/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CustomFactor.cpp
 * @brief   Class to enable arbitrary factors with runtime swappable error function.
 * @author  Fan Jiang
 */

#include <gtsam/nonlinear/CustomFactor.h>

namespace gtsam {

/*
 * Calculates the unwhitened error by invoking the callback functor (i.e. from Python).
 */
Vector CustomFactor::unwhitenedError(const Values& x, OptionalMatrixVecType H) const {
  if(this->active(x)) {

    if(H) {
      /*
       * In this case, we pass the raw pointer to the `std::vector<Matrix>` object directly to pybind.
       * As the type `std::vector<Matrix>` has been marked as opaque in `preamble.h`, any changes in
       * Python will be immediately reflected on the C++ side.
       *
       * Example:
       * ```
       * def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
       *    <calculated error>
       *    if not H is None:
       *        <calculate the Jacobian>
       *        H[0] = J1
       *        H[1] = J2
       *        ...
       *    return error
       * ```
       */
      return this->error_function_(*this, x, H.get_ptr());
    } else {
      /*
       * In this case, we pass the a `nullptr` to pybind, and it will translate to `None` in Python.
       * Users can check for `None` in their callback to determine if the Jacobian is requested.
       */
      return this->error_function_(*this, x, nullptr);
    }
  } else {
    return Vector::Zero(this->dim());
  }
}

void CustomFactor::print(const std::string &s, const KeyFormatter &keyFormatter) const {
  std::cout << s << "CustomFactor on ";
  auto keys_ = this->keys();
  bool f = false;
  for (const Key &k: keys_) {
    if (f)
      std::cout << ", ";
    std::cout << keyFormatter(k);
    f = true;
  }
  std::cout << "\n";
  if (this->noiseModel_)
    this->noiseModel_->print("  noise model: ");
  else
    std::cout << "no noise model" << std::endl;
}

}

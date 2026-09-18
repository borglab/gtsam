/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CustomFactor.h
 * @brief   Class to enable arbitrary factors with runtime swappable error function.
 * @author  Fan Jiang
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <functional>
#include <optional>

namespace gtsam {

using JacobianVector = std::vector<Matrix>;

class CustomFactor;

/// Jacobians handed to the Python callback, absent when only the error is wanted.
using OptionalJacobianVector = std::optional<std::reference_wrapper<JacobianVector>>;

/*
 * NOTE
 * ==========
 * pybind11 will invoke a copy if this is `JacobianVector &`, and modifications in Python will not be reflected.
 *
 * A `std::optional<std::reference_wrapper<JacobianVector>>` keeps reference semantics -- `JacobianVector` is
 * opaque (see `preamble/custom.h`), so pybind11 maintains the `std::vector` memory layout and Python's writes
 * are visible in C++ -- while also telling pybind11 that the argument is nullable. A bare pointer does not:
 * pybind11 renders pointer arguments without `Optional`, so the generated stub claims the callback always
 * receives a `JacobianVector`, and a correct `if H is not None` guard gets reported as unreachable.
 */
using CustomErrorFunction = std::function<Vector(const CustomFactor &, const Values &, OptionalJacobianVector)>;

/**
 * @brief Custom factor that takes a std::function as the error
 * @addtogroup nonlinear
 * \nosubgrouping
 *
 * This factor is mainly for creating a custom factor in Python.
 */
class GTSAM_EXPORT CustomFactor: public NoiseModelFactor {
protected:
  CustomErrorFunction error_function_;

protected:

  using Base = NoiseModelFactor;
  using This = CustomFactor;

public:

  /**
   * Default Constructor for I/O
   */
  CustomFactor() = default;

  /**
   * Constructor
   * @param noiseModel shared pointer to noise model
   * @param keys keys of the variables
   * @param errorFunction the error functional
   */
  CustomFactor(const SharedNoiseModel &noiseModel, const KeyVector &keys, const CustomErrorFunction &errorFunction) :
      Base(noiseModel, keys) {
    this->error_function_ = errorFunction;
  }

  /**
    * Calls the errorFunction closure, which is a std::function object
    * One can check if a derivative is needed in the errorFunction by checking the length of Jacobian array
    */
  Vector unwhitenedError(const Values &x, OptionalMatrixVecType H = nullptr) const override;

  /** print */
  void print(const std::string &s,
             const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override;

  /**
   * Mark not sendable
   */
  bool sendable() const override {
    return false;
  }

private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("CustomFactor",
                                        boost::serialization::base_object<Base>(*this));
  }
#endif
};

}

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

/**
 * @file UnaryMeasurement.h
 * @author Akshay Krishnan
 * @date January 2026
 * @brief Unary measurement represents a measurement on a single key in a graph.
 * It mirrors BinaryMeasurement but holds just one key. The measurement value
 * and noise model are stored without an accompanying error function.
 */

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>

#include <iostream>
#include <memory>
#include <vector>

namespace gtsam {

/**
 * @brief Unary measurement represents a measurement on a single key in a graph.
 */
template <class T>
class UnaryMeasurement : public Factor {
  // Check that T type is testable
  GTSAM_CONCEPT_ASSERT(IsTestable<T>);

 public:
  // shorthand for a smart pointer to a measurement
  using shared_ptr = std::shared_ptr<UnaryMeasurement>;

 private:
  T measured_;                   ///< The measurement
  SharedNoiseModel noiseModel_;  ///< Noise model

 public:
  /**
   * @brief Constructs a unary measurement with a key, value, and noise model.
   * @param key The key of the measurement.
   * @param measured The measurement value.
   * @param model The noise model (optional).
   */
  UnaryMeasurement(Key key, const T &measured,
                   const SharedNoiseModel &model = nullptr)
      : Factor(std::vector<Key>({key})),
        measured_(measured),
        noiseModel_(noiseModel::validOrDefault(measured, model)) {}

  /// @name Standard Interface
  /// @{

  // Returns the key of the measurement.
  Key key() const { return keys_[0]; }
  // Returns the measurement value.
  const T &measured() const { return measured_; }
  // Returns the noise model.
  const SharedNoiseModel &noiseModel() const { return noiseModel_; }

  /// @}
  /// @name Testable
  /// @{

  // Prints the measurement.
  void print(const std::string &s, const KeyFormatter &keyFormatter =
                                       DefaultKeyFormatter) const override {
    std::cout << s << "UnaryMeasurement(" << keyFormatter(this->key()) << ")\n";
    traits<T>::Print(measured_, "  measured: ");
    if (noiseModel_)
      noiseModel_->print("  noise model: ");
    else
      std::cout << "  noise model: (null)\n";
  }

  // Checks if the measurement is equal to another measurement.
  bool equals(const UnaryMeasurement &expected, double tol = 1e-9) const {
    const UnaryMeasurement<T> *e =
        dynamic_cast<const UnaryMeasurement<T> *>(&expected);
    return e != nullptr && Factor::equals(*e) &&
           traits<T>::Equals(this->measured_, e->measured_, tol) &&
           ((!noiseModel_ && !expected.noiseModel_) ||
            (noiseModel_ && noiseModel_->equals(*expected.noiseModel())));
  }
  /// @}
};
}  // namespace gtsam

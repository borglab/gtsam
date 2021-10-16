/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

/**
 * @file BinaryMeasurement.h
 * @author Akshay Krishnan
 * @date July 2020
 * @brief Binary measurement represents a measurement between two keys in a
 * graph. A binary measurement is similar to a BetweenFactor, except that it
 * does not contain an error function. It is a measurement (along with a noise
 * model) from one key to another. Note that the direction is important. A
 * measurement from key1 to key2 is not the same as the same measurement from
 * key2 to key1.
 */

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>

#include <iostream>
#include <vector>

namespace gtsam {

template <class T> class BinaryMeasurement : public Factor {
  // Check that T type is testable
  BOOST_CONCEPT_ASSERT((IsTestable<T>));

public:
  // shorthand for a smart pointer to a measurement
  using shared_ptr = typename std::shared_ptr<BinaryMeasurement>;

private:
  T measured_;                  ///< The measurement
  SharedNoiseModel noiseModel_; ///< Noise model

 public:
  BinaryMeasurement(Key key1, Key key2, const T &measured,
                    const SharedNoiseModel &model = nullptr)
      : Factor(std::vector<Key>({key1, key2})),
        measured_(measured),
        noiseModel_(model) {}

  /// Destructor
  virtual ~BinaryMeasurement() {}

  /// @name Standard Interface
  /// @{

  Key key1() const { return keys_[0]; }
  Key key2() const { return keys_[1]; }
  const T &measured() const { return measured_; }
  const SharedNoiseModel &noiseModel() const { return noiseModel_; }

  /// @}
  /// @name Testable
  /// @{

  void print(const std::string &s, const KeyFormatter &keyFormatter =
                                       DefaultKeyFormatter) const override {
    std::cout << s << "BinaryMeasurement(" << keyFormatter(this->key1()) << ","
              << keyFormatter(this->key2()) << ")\n";
    traits<T>::Print(measured_, "  measured: ");
    this->noiseModel_->print("  noise model: ");
  }

  bool equals(const BinaryMeasurement &expected, double tol = 1e-9) const {
    const BinaryMeasurement<T> *e =
        dynamic_cast<const BinaryMeasurement<T> *>(&expected);
    return e != nullptr && Factor::equals(*e) &&
           traits<T>::Equals(this->measured_, e->measured_, tol) &&
           noiseModel_->equals(*expected.noiseModel());
  }
  /// @}
};
} // namespace gtsam

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
 * @brief Binary measurement for representing edges in the epipolar graph
 */

#include <ostream>


#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>

namespace gtsam {

template<class VALUE> 
class BinaryMeasurement {
    // Check that VALUE type is testable
    BOOST_CONCEPT_ASSERT((IsTestable<VALUE>));

public:
    typedef VALUE T;

    // shorthand for a smart pointer to a measurement
    typedef typename boost::shared_ptr<BinaryMeasurement> shared_ptr;

private:
    Key key1_, key2_;   /** Keys */

    VALUE measured_; /** The measurement */

    SharedNoiseModel noiseModel_; /** Noise model */

public:
    /** default constructor - only use for serialization */
    BinaryMeasurement() {}

    /** Constructor */
    BinaryMeasurement(Key key1, Key key2, const VALUE& measured,
        const SharedNoiseModel& model = nullptr) :
      key1_(key1), key2_(key2), measured_(measured), noiseModel_(model) {
    }

    virtual ~BinaryMeasurement() {}

    Key key1() const { return key1_;  }

    Key key2() const { return key2_;  }

    const SharedNoiseModel& noiseModel() const { return noiseModel_; }

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BinaryMeasurement("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      traits<T>::Print(measured_, "  measured: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const BinaryMeasurement& expected, double tol=1e-9) const {
      const BinaryMeasurement<VALUE> *e =  dynamic_cast<const  BinaryMeasurement<VALUE>*> (&expected);
      return e != nullptr && key1_ == e->key1_ && 
             key2_ == e->key2_
             && traits<VALUE>::Equals(this->measured_, e->measured_, tol) && 
             noiseModel_->equals(expected.noiseModel());
    }

    /** return the measured */
    VALUE measured() const {
      return measured_;
    }
  }; // \class BetweenMeasurement
}
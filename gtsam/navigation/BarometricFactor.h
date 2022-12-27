/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   BarometricFactor.h
 *  @author Peter Milani
 *  @brief  Header file for Barometric factor
 *  @date   December 16, 2021
 **/
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Prior on height in a cartesian frame.
 * Receive barometric pressure in kilopascals
 * Model with a slowly moving bias to capture differences
 * between the height and the standard atmosphere
 * https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
 * @ingroup navigation
 */
class GTSAM_EXPORT BarometricFactor : public NoiseModelFactorN<Pose3, double> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(BarometricFactor, 2);

   private:
    typedef NoiseModelFactorN<Pose3, double> Base;

    double nT_;  ///< Height Measurement based on a standard atmosphere

   public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<BarometricFactor> shared_ptr;

    /// Typedef to this class
    typedef BarometricFactor This;

    /** default constructor - only use for serialization */
    BarometricFactor() : nT_(0) {}

    ~BarometricFactor() override {}

    /**
     * @brief Constructor from a measurement of pressure in KPa.
     * @param key of the Pose3 variable that will be constrained
     * @param key of the barometric bias that will be constrained
     * @param baroIn measurement in KPa
     * @param model Gaussian noise model 1 dimension
     */
    BarometricFactor(Key key, Key baroKey, const double& baroIn,
                     const SharedNoiseModel& model)
        : Base(model, key, baroKey), nT_(heightOut(baroIn)) {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /// print
    void print(
        const std::string& s = "",
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

    /// equals
    bool equals(const NonlinearFactor& expected,
                double tol = 1e-9) const override;

    /// vector of errors
    Vector evaluateError(
        const Pose3& p, const double& b,
        boost::optional<Matrix&> H = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const override;

    inline const double& measurementIn() const { return nT_; }

    inline double heightOut(double n) const {
        // From https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
        return (std::pow(n / 101.29, 1. / 5.256) * 288.08 - 273.1 - 15.04) /
               -0.00649;
    };

    inline double baroOut(const double& meters) {
        double temp = 15.04 - 0.00649 * meters;
        return 101.29 * std::pow(((temp + 273.1) / 288.08), 5.256);
    };

   private:
    /// Serialization function
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
        // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
        ar& boost::serialization::make_nvp(
            "NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
        ar& BOOST_SERIALIZATION_NVP(nT_);
    }
};

}  // namespace gtsam

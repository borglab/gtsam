/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConstantVelocityFactor.h
 * @brief   Maintain a constant velocity motion model between two NavStates
 * @author  Asa Hammond
 */

#pragma once

#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Binary factor for applying a constant velocity model to a moving body represented as a NavState.
 * The only measurement is dt, the time delta between the states.
 */
class ConstantVelocityFactor : public NoiseModelFactorN<NavState, NavState>,
                               public DeprecatedFactorAliases<NavState, NavState> {

    double dt_;

   public:
    using Base = NoiseModelFactorN<NavState, NavState>;

   public:
    ConstantVelocityFactor(Key i, Key j, double dt, const SharedNoiseModel &model)
        : NoiseModelFactorN<NavState, NavState>(model, i, j), dt_(dt) {}
    ~ConstantVelocityFactor() override{};

    /**
     * @brief Caclulate error: (x2 - x1.update(dt)))
     * where X1 and X1 are NavStates and dt is
     * the time difference in seconds between the states.
     * @param x1 NavState for key a
     * @param x2 NavState for key b
     * @param H1 optional jacobian in x1
     * @param H2 optional jacobian in x2
     * @return * Vector
     */
    gtsam::Vector evaluateError(const NavState &x1, const NavState &x2,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
        // only used to use update() below
        static const Vector3 b_accel{0.0, 0.0, 0.0};
        static const Vector3 b_omega{0.0, 0.0, 0.0};

        Matrix99 predicted_H_x1;
        NavState predicted = x1.update(b_accel, b_omega, dt_, H1 ? &predicted_H_x1 : nullptr, {}, {});

        Matrix99 error_H_predicted;
        Vector9 error = predicted.localCoordinates(x2, H1 ? &error_H_predicted : nullptr, H2);

        if (H1) {
            *H1 = error_H_predicted * predicted_H_x1;
        }
        return error;
    }
};

}  // namespace gtsam

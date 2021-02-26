#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Binary factor for applying a constant velocity model to a moving body represented as a NavState.
 * The only measurement is dt, the time delta between the states.
 */
class ConstantVelocityFactor : public NoiseModelFactor2<NavState, NavState> {
    double dt_;

   public:
    using Base = NoiseModelFactor2<NavState, NavState>;

   public:
    ConstantVelocityFactor(Key i, Key j, double dt, const SharedNoiseModel &model)
        : NoiseModelFactor2<NavState, NavState>(model, i, j), dt_(dt) {}
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

        Matrix predicted_H_x1;
        NavState predicted = x1.update(b_accel, b_omega, dt_, predicted_H_x1, {}, {});

        Matrix error_H_predicted;
        Vector9 error = predicted.localCoordinates(x2, error_H_predicted, H2);

        if (H1) {
            *H1 = error_H_predicted * predicted_H_x1;
        }
        return error;
    }
};

}  // namespace gtsam

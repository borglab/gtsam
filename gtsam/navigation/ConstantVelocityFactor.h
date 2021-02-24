#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class ConstantVelocityFactor : public NoiseModelFactor2<NavState, NavState> {
    double dt_;

   public:
    using Base = NoiseModelFactor2<NavState, NavState>;

   public:
    ConstantVelocityFactor(Key i, Key j, double dt, const SharedNoiseModel &model)
        : NoiseModelFactor2<NavState, NavState>(model, i, j), dt_(dt) {}
    ~ConstantVelocityFactor() override{};

    gtsam::Vector evaluateError(const NavState &x1, const NavState &x2,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
        if (H1) {
            (*H1) = numericalDerivative21<gtsam::Vector, NavState, NavState>(
                boost::bind(ConstantVelocityFactor::evaluateError_, _1, _2, dt_), x1, x2, 1e-5);
        }
        if (H2) {
            (*H2) = numericalDerivative22<gtsam::Vector, NavState, NavState>(
                boost::bind(ConstantVelocityFactor::evaluateError_, _1, _2, dt_), x1, x2, 1e-5);
        }

        return evaluateError_(x1, x2, dt_);
    }

   private:
    static gtsam::Vector evaluateError_(const NavState &x1, const NavState &x2, double dt) {
        const Velocity3 &v1 = x1.v();
        const Velocity3 &v2 = x2.v();
        const Point3 &p1 = x1.t();
        const Point3 &p2 = x2.t();

        // trapezoidal integration constant accelleration
        // const Point3 hx = p1 + Point3((v1 + v2) * dt / 2.0);

        // euler start
        // const Point3 hx = p1 + Point3(v1 * dt);

        // euler end
        const Point3 hx = p1 + Point3(v2 * dt);

        return p2 - hx;
    }
};

}  // namespace gtsam

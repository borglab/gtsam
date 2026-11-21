/**
 * @file  IMUScenarios.h
 * @brief Filter-agnostic ground-truth trajectory factory shared by the
 *        tg_eqf, tfg_inekf, and mekf example runners.
 *
 * Each factory returns a concrete gtsam::Scenario describing one canonical
 * trajectory. Defining them in one place guarantees every filter is driven
 * through the IDENTICAL ground truth (same speeds, rates, accelerations), so a
 * cross-filter comparison reflects filter behaviour only -- not incidental
 * differences in how each example wired up its scenario.
 *
 * Process-noise PSDs for the runners live in ImuProcessNoise.h (same CLI → PSD
 * mapping for tg_eqf, tfg_inekf, and mekf).
 *
 * Coordinate frame: the runners drive these trajectories through a Z-down (NED)
 * navigation frame -- see imu_scenarios::navGravity() in ScenarioHarness.h,
 * which is the single place that choice is made. So nav +z points DOWN: a
 * positive z position is depth below the surface, a positive nav-z acceleration
 * is a descent, and a positive body-z angular rate is a right (clockwise seen
 * from above) turn. The kinematics below are frame-agnostic; only the comments
 * name a direction.
 *
 * Usage:
 *   const auto scenario = imu_scenarios::vertical();
 *   runScenario(scenario, opts, "Vertical");
 *
 * The returned object is a concrete Scenario subclass; bind it to a named local
 * (so its lifetime spans the runScenario call) and pass by const reference.
 */
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam_unstable/examples_common/SamoaWaypoints.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unsupported/Eigen/Splines>
#include <vector>

namespace imu_scenarios {

// --- Canonical trajectory parameters (shared across all filters) ------------
// Kept here, not in the individual example mains, so every filter sees the same
// numbers. Adjust once to retune every runner consistently.
inline constexpr double kVertAccel = 0.2;   // Vertical: m/s^2 down (nav +z)
inline constexpr double kVertSpeed0 = 0.0;  // Vertical: initial m/s

inline constexpr double kNavCruiseSpeed =
    2.0;  // TypicalNavigation: cruise m/s, body x
inline constexpr double kNavTurnRate =
    0.4;  // TypicalNavigation: yaw rate during a turn, rad/s

inline constexpr double kSplinePathDuration =
    40.0;  // WaypointSpline: warped-time seconds to traverse the waypoint list
           // (u:0->1). Sized > the default 30 s --duration so a default run
           // never reaches the final-waypoint clamp; see SplineScenario.

inline constexpr double kSamoaPathDuration =
    300.0;  // SamoaSurvey: warped-time seconds to traverse the real-trajectory
            // waypoint list, matching the ~300 s of real data they were
            // extracted from (see SamoaWaypoints.h).

// Every scenario is required to start at rest (v=0, a=0) so filters can be
// initialized at the scenario's true initial belief, offset by the configured
// init sigmas (see sampleInitialEstimate in ScenarioHarness.h) rather than a
// hand-tuned one. Vertical is an AcceleratingScenario that already ramps
// velocity linearly from v0, so setting its initial-speed constant to 0 above
// suffices. TypicalNavigation and WaypointSpline instead have a constant
// (nonzero) cruise speed by construction, so they reach it via a smooth
// spin-up over kRampDuration; see evalRamp below.
inline constexpr double kRampDuration = 2.0;  // s, shared spin-up duration

/// Smooth spin-up profile shared by every constant-speed scenario. `S(t)` is an
/// arc-length reparameterization: `S(0)=0`, `S'(0)=S''(0)=0` (fully at rest),
/// and for `t >= T` it reduces to `S(t) = t - T/2` with `S'=1, S''=0` (exact
/// cruise, C2-continuous stitch at `t=T`). Built from the quintic smootherstep
/// `f(x) = 6x^5 - 15x^4 + 10x^3` (`S' = f(t/T)`, `S = integral of f`, `S'' =
/// f'/T`). https://iquilezles.org/articles/smoothsteps/
struct RampProfile {
  double S;
  double Sdot;
  double Sddot;
};

inline RampProfile evalRamp(double t, double T) {
  if (t <= 0.0) {
    return RampProfile{0.0, 0.0, 0.0};
  }
  if (t >= T) {
    return RampProfile{t - T / 2.0, 1.0, 0.0};
  }
  const double x = t / T;
  const double x2 = x * x, x3 = x2 * x, x4 = x3 * x, x5 = x4 * x, x6 = x5 * x;
  return RampProfile{T * (x6 - 3.0 * x5 + 2.5 * x4),
                     6.0 * x5 - 15.0 * x4 + 10.0 * x3,
                     (30.0 * x4 - 60.0 * x3 + 30.0 * x2) / T};
}

/// Waypoint trajectory: a cubic C2 spline interpolated through a list of
/// nav-frame position waypoints, driving position/velocity/acceleration by
/// ANALYTIC differentiation of that one spline (so the three channels are
/// exactly self-consistent, as ScenarioRunner's IMU synthesis requires --
/// unlike gtsam::DiscreteScenario, which interpolates each channel
/// independently). The spline parameter u in [0,1] is mapped to physical time
/// through the shared `evalRamp` time-warp `S(t)` (u = S(t) / pathDuration), so
/// the path spins up smoothly from rest -- pose(0)=origin, v(0)=a(0)=0 --
/// matching the rest-start invariant every other factory obeys.
///
/// Attitude is OPTIONAL. With no attitudes given, rotation is held at
/// identity and omega_b = 0, as before. Given one (roll, pitch, yaw) triple
/// per waypoint (aerospace 3-2-1, R(u) = Rz(yaw) * Ry(pitch) * Rx(roll), body
/// to nav; yaw already unwrapped by the caller), the three angle channels get
/// their own cubic C2 spline over the SAME u as the position spline, and
/// omega_b is the analytic body rate from the Euler-angle rates -- so the
/// synthesized gyro stays exactly consistent with pose(t).rotation(). udot(0)
/// = 0, so omega_b(0) = 0 regardless of attitude, preserving the rest-start
/// invariant.
///
/// The map from waypoint index to spline parameter u defaults to chord
/// length (the waypoints describe a shape, not a schedule). Pass
/// KnotParameterization::Uniform when the waypoints are already spaced
/// uniformly in time, so u stays proportional to time instead of arc length.
///
/// Past the final waypoint (u >= 1) the pose is clamped and held at rest
/// (attitude included); size pathDuration so a run stays within the path
/// (the default does).
class SplineScenario : public gtsam::Scenario {
 public:
  /// How waypoint index maps to spline parameter u.
  enum class KnotParameterization {
    ChordLength,  ///< u_i from cumulative distance between waypoints.
    Uniform,      ///< u_i = i / (n - 1); use when waypoints are evenly
                  ///< spaced in time rather than in arc length.
  };

  /// @param waypoints  >= 2 nav-frame positions; the path is auto-translated
  ///                   so pose(0) sits at the origin.
  /// @param attitudes  empty for identity attitude, or one (roll, pitch, yaw)
  ///                   triple per waypoint, same length and order as
  ///                   waypoints.
  /// @param pathDuration  warped-time seconds to traverse waypoints.front() ->
  ///                   waypoints.back() (spline parameter u: 0 -> 1). Must be
  ///                   > 0. Larger => slower traversal.
  /// @param rampDuration  rest-to-cruise spin-up (shared evalRamp profile).
  /// @param knotParam  how waypoint index maps to u; shared by the position
  ///                   and attitude splines.
  SplineScenario(const std::vector<gtsam::Vector3>& waypoints,
                 const std::vector<gtsam::Vector3>& attitudes,
                 double pathDuration, double rampDuration = kRampDuration,
                 KnotParameterization knotParam =
                     KnotParameterization::ChordLength)
      : pathDuration_(pathDuration), rampDuration_(rampDuration) {
    const std::size_t n = waypoints.size();
    if (n < 2) {
      throw std::invalid_argument("SplineScenario needs at least 2 waypoints.");
    }
    if (!attitudes.empty() && attitudes.size() != n) {
      throw std::invalid_argument(
          "SplineScenario attitudes must be empty or match waypoints in "
          "size.");
    }
    if (!(pathDuration_ > 0.0)) {
      throw std::invalid_argument("SplineScenario pathDuration must be > 0.");
    }
    Eigen::Array<double, 3, Eigen::Dynamic> pts(3, n);
    for (std::size_t i = 0; i < n; ++i) {
      pts.col(i) = waypoints[i].array();
    }
    // Cubic gives a C2 (continuous-acceleration) path; drop the degree only
    // when there are too few waypoints to support it.
    const Eigen::Index degree =
        std::min<Eigen::Index>(3, static_cast<Eigen::Index>(n) - 1);
    Eigen::SplineFitting<Spline3>::KnotVectorType uParams;
    if (knotParam == KnotParameterization::Uniform) {
      uParams.resize(static_cast<Eigen::Index>(n));
      for (std::size_t i = 0; i < n; ++i) {
        uParams(static_cast<Eigen::Index>(i)) =
            static_cast<double>(i) / static_cast<double>(n - 1);
      }
    } else {
      Eigen::ChordLengths(pts, uParams);
    }
    spline_ = Eigen::SplineFitting<Spline3>::Interpolate(pts, degree, uParams);
    origin_ = spline_(0.0).matrix();              // == waypoints.front()
    finalPos_ = spline_(1.0).matrix() - origin_;  // held past the last waypoint

    hasAttitude_ = !attitudes.empty();
    if (hasAttitude_) {
      Eigen::Array<double, 3, Eigen::Dynamic> angles(3, n);
      for (std::size_t i = 0; i < n; ++i) {
        angles.col(i) = attitudes[i].array();
      }
      attitudeSpline_ =
          Eigen::SplineFitting<Spline3>::Interpolate(angles, degree, uParams);
      attitudeOrigin_ = attitudeSpline_(0.0).matrix();
      attitudeFinal_ = attitudeSpline_(1.0).matrix();
    }
  }

  gtsam::Pose3 pose(double t) const override {
    return gtsam::Pose3(rotation_n(t), gtsam::Point3(position_n(t)));
  }
  gtsam::Vector3 omega_b(double t) const override {
    if (!hasAttitude_) {
      return gtsam::Vector3::Zero();
    }
    const UParam u = mapTime(t);
    if (u.clamped) {
      return gtsam::Vector3::Zero();
    }
    const auto der = attitudeSpline_.derivatives(u.u, 1);
    const double phi = der(0, 0), theta = der(1, 0);
    const double phidot = der(0, 1) * u.udot;
    const double thetadot = der(1, 1) * u.udot;
    const double psidot = der(2, 1) * u.udot;
    const double wx = phidot - psidot * std::sin(theta);
    const double wy =
        thetadot * std::cos(phi) + psidot * std::cos(theta) * std::sin(phi);
    const double wz =
        -thetadot * std::sin(phi) + psidot * std::cos(theta) * std::cos(phi);
    return gtsam::Vector3(wx, wy, wz);
  }
  gtsam::Vector3 velocity_n(double t) const override {
    const UParam u = mapTime(t);
    if (u.clamped) {
      return gtsam::Vector3::Zero();
    }
    return dpdu(u.u) * u.udot;
  }
  gtsam::Vector3 acceleration_n(double t) const override {
    const UParam u = mapTime(t);
    if (u.clamped) {
      return gtsam::Vector3::Zero();
    }
    const auto der = spline_.derivatives(u.u, 2);
    const gtsam::Vector3 d1 = der.col(1).matrix();
    const gtsam::Vector3 d2 = der.col(2).matrix();
    return d2 * (u.udot * u.udot) + d1 * u.uddot;
  }

 private:
  typedef Eigen::Spline<double, 3> Spline3;

  gtsam::Vector3 position_n(double t) const {
    const UParam u = mapTime(t);
    return u.clamped && u.u >= 1.0 ? finalPos_
                                   : (spline_(u.u).matrix() - origin_);
  }
  gtsam::Vector3 dpdu(double u) const {
    return spline_.derivatives(u, 1).col(1).matrix();
  }
  gtsam::Rot3 rotation_n(double t) const {
    if (!hasAttitude_) {
      return gtsam::Rot3();
    }
    const UParam u = mapTime(t);
    const gtsam::Vector3 rpy = u.clamped
        ? (u.u >= 1.0 ? attitudeFinal_ : attitudeOrigin_)
        : attitudeSpline_(u.u).matrix();
    return gtsam::Rot3::Ypr(rpy(2), rpy(1), rpy(0));
  }

  /// Spline parameter u = S(t)/pathDuration and its time derivatives, where S
  /// is the shared rest-start ramp. `clamped` marks t <= 0 (held at origin) or
  /// u >= 1 (held at the final waypoint) -- both rest states (v = a = 0).
  struct UParam {
    double u, udot, uddot;
    bool clamped;
  };
  UParam mapTime(double t) const {
    const RampProfile r = evalRamp(t, rampDuration_);
    const double u = r.S / pathDuration_;
    if (u <= 0.0) {
      return {0.0, 0.0, 0.0, true};
    }
    if (u >= 1.0) {
      return {1.0, 0.0, 0.0, true};
    }
    return {u, r.Sdot / pathDuration_, r.Sddot / pathDuration_, false};
  }

  const double pathDuration_, rampDuration_;
  Spline3 spline_;
  gtsam::Vector3 origin_ = gtsam::Vector3::Zero();
  gtsam::Vector3 finalPos_ = gtsam::Vector3::Zero();
  bool hasAttitude_ = false;
  Spline3 attitudeSpline_;
  gtsam::Vector3 attitudeOrigin_ = gtsam::Vector3::Zero();
  gtsam::Vector3 attitudeFinal_ = gtsam::Vector3::Zero();
};

/// One leg of a piecewise-constant-twist ground path: constant body angular
/// rate `w` and body-frame velocity `v_body`, held for `duration_s` seconds.
struct NavLeg {
  double duration_s;
  gtsam::Vector3 w;
  gtsam::Vector3 v_body;
};

/// "Typical navigation": ramps up from rest to cruise speed (via the shared
/// evalRamp spin-up), then drives a sequence of legs -- straight cruise with
/// occasional coordinated turns (yaw rate at constant forward
/// speed), alternating direction, the way a car follows a road with corners.
/// Each leg is an exact ConstantTwistScenario segment chained onto the
/// accumulated end pose of the previous one, so position/velocity stay
/// continuous across leg boundaries; only the body angular rate steps between
/// legs (no extra smoothing beyond the initial rest-to-cruise ramp). Past the
/// last leg the leg sequence repeats from the first leg (the ramp is not
/// repeated), each cycle chained onto the accumulated end pose of the
/// previous one, so navState(t) stays well defined -- and keeps exercising
/// turns -- for any --duration. The seam is smooth: last and first leg share
/// the same body twist (straight cruise), so position/velocity are continuous
/// across cycles just as across legs.
class TypicalNavigationScenario : public gtsam::Scenario {
 public:
  TypicalNavigationScenario(double rampDuration, double cruiseSpeed,
                            std::vector<NavLeg> legs)
      : rampDuration_(rampDuration),
        cruiseVelocityBody_(cruiseSpeed, 0.0, 0.0),
        legs_(std::move(legs)) {
    rampEndPose_ = rampPose(rampDuration_);
    double t = 0.0;     // leg times relative to ramp end
    gtsam::Pose3 pose;  // leg poses relative to ramp end
    for (const auto& leg : legs_) {
      legStartTimes_.push_back(t);
      legStartPoses_.push_back(pose);
      const gtsam::Vector6 twist =
          (gtsam::Vector6() << leg.w, leg.v_body).finished();
      pose = pose * gtsam::Pose3::Expmap(twist * leg.duration_s);
      t += leg.duration_s;
    }
    cycleDuration_ = t;
    cycleDelta_ = pose;
  }

  gtsam::Pose3 pose(double t) const override {
    if (t <= rampDuration_) {
      return rampPose(t);
    }
    size_t cycles;
    const double tau = cycleTime(t, &cycles);
    const size_t i = legIndex(tau);
    const NavLeg& leg = legs_[i];
    const gtsam::Vector6 twist =
        (gtsam::Vector6() << leg.w, leg.v_body).finished();
    gtsam::Pose3 cycleStart = rampEndPose_;
    for (size_t k = 0; k < cycles; ++k) {
      cycleStart = cycleStart * cycleDelta_;
    }
    return cycleStart * legStartPoses_[i] *
           gtsam::Pose3::Expmap(twist * (tau - legStartTimes_[i]));
  }
  gtsam::Vector3 omega_b(double t) const override {
    if (t <= rampDuration_) {
      return gtsam::Vector3::Zero();
    }
    return legs_[legIndex(cycleTime(t))].w;
  }
  gtsam::Vector3 velocity_n(double t) const override {
    if (t <= rampDuration_) {
      return rotation(t).matrix() * cruiseVelocityBody_ *
             evalRamp(t, rampDuration_).Sdot;
    }
    return rotation(t).matrix() * legs_[legIndex(cycleTime(t))].v_body;
  }
  gtsam::Vector3 acceleration_n(double t) const override {
    if (t <= rampDuration_) {
      return rotation(t) *
             (cruiseVelocityBody_ * evalRamp(t, rampDuration_).Sddot);
    }
    const NavLeg& leg = legs_[legIndex(cycleTime(t))];
    return rotation(t) * leg.w.cross(leg.v_body);
  }

 private:
  gtsam::Pose3 rampPose(double t) const {
    const gtsam::Vector6 twist =
        (gtsam::Vector6() << gtsam::Vector3::Zero(), cruiseVelocityBody_)
            .finished();
    return gtsam::Pose3::Expmap(twist * evalRamp(t, rampDuration_).S);
  }

  /// Time within the current leg cycle, in [0, cycleDuration_). Optionally
  /// reports how many full cycles precede t (for chaining cycle end poses).
  double cycleTime(double t, size_t* cycles = nullptr) const {
    const double s = t - rampDuration_;
    double n = std::floor(s / cycleDuration_);
    double tau = s - n * cycleDuration_;
    if (tau >= cycleDuration_) {  // guard the floating-point edge at a seam
      tau -= cycleDuration_;
      n += 1.0;
    }
    if (cycles) {
      *cycles = static_cast<size_t>(n);
    }
    return tau;
  }

  size_t legIndex(double tau) const {
    for (size_t i = 0; i + 1 < legs_.size(); ++i) {
      if (tau < legStartTimes_[i + 1]) {
        return i;
      }
    }
    return legs_.size() - 1;
  }

  const double rampDuration_;
  const gtsam::Vector3 cruiseVelocityBody_;
  const std::vector<NavLeg> legs_;
  gtsam::Pose3 rampEndPose_;
  double cycleDuration_ = 0.0;
  gtsam::Pose3
      cycleDelta_;  ///< end pose of one leg cycle, relative to its start
  std::vector<double> legStartTimes_;        ///< relative to ramp end
  std::vector<gtsam::Pose3> legStartPoses_;  ///< relative to ramp end
};

// --- Factories --------------------------------------------------------------

/// Vertical: a pure vertical dive -- acceleration straight down the nav z axis
/// (accel-bias vs gravity separability along the gravity axis), zero rotation,
/// starting from rest. Depth therefore grows throughout the run.
inline gtsam::AcceleratingScenario vertical() {
  const gtsam::Vector3 v0(0.0, 0.0, kVertSpeed0);
  const gtsam::Vector3 a_n(0.0, 0.0, kVertAccel);
  return gtsam::AcceleratingScenario(gtsam::Rot3(), gtsam::Point3(0, 0, 0), v0,
                                     a_n, gtsam::Vector3::Zero());
}

/// Default waypoint list for waypointSpline(): a weaving 3-D path (lateral
/// S-bends with a gentle dive and rise) starting at the origin, so the spline
/// exercises all three position/velocity/acceleration channels at once. With +z
/// down the path descends to 2.5 m, then rises back through the start depth to
/// 1 m above it before returning to the surface.
inline std::vector<gtsam::Vector3> defaultSplineWaypoints() {
  return {
      gtsam::Vector3(0.0, 0.0, 0.0),    gtsam::Vector3(10.0, 6.0, 1.0),
      gtsam::Vector3(22.0, -4.0, 2.5),  gtsam::Vector3(34.0, 5.0, 1.0),
      gtsam::Vector3(46.0, -6.0, -1.0), gtsam::Vector3(58.0, 2.0, 0.5),
      gtsam::Vector3(70.0, 0.0, 0.0),
  };
}

/// Waypoint spline: a smooth cubic path interpolated through predefined 3-D
/// waypoints, spun up from rest (see SplineScenario). Identity attitude, so it
/// stresses the translational channels and the position/DVL/depth aiding
/// path.
inline SplineScenario waypointSpline(
    const std::vector<gtsam::Vector3>& waypoints = defaultSplineWaypoints(),
    double pathDuration = kSplinePathDuration) {
  return SplineScenario(waypoints, {}, pathDuration);
}

/// Samoa survey: a waypoint spline through the first 300 s of a real AUV
/// survey trajectory (see SamoaWaypoints.h), spun up from rest via the shared
/// evalRamp profile like every other scenario. The waypoints are generated
/// already moved into the inertial frame -- first waypoint at the origin,
/// initial direction of travel along nav +x, gravity axis untouched -- so
/// this starts at rest at the origin like every other factory. Real attitude
/// (samoaSurveyAttitudes()) drives omega_b, so this exercises the gyro
/// channel too, not just translation. The waypoints are spaced uniformly in
/// TIME (one every ~0.2 s of the source data), not by arc length, so this
/// uses KnotParameterization::Uniform -- chord-length knots would compress
/// slow stretches of the real trajectory (e.g. turns) in u, distorting the
/// rates SplineScenario derives from du/dt. Fixed duration: past the last
/// waypoint the path clamps and holds at rest (SplineScenario), so a longer
/// --duration run just idles after 300 s.
inline SplineScenario samoaSurvey() {
  return SplineScenario(samoaSurveyWaypoints(), samoaSurveyAttitudes(),
                        kSamoaPathDuration, kRampDuration,
                        SplineScenario::KnotParameterization::Uniform);
}

/// Typical navigation: rest -> cruise, straight legs with two occasional
/// coordinated turns in opposite directions, the way a car follows a road.
/// Sized to fill the default 30 s --duration (2 s ramp + 28 s of legs) but
/// works at any duration -- shorter runs just see a truncated route, longer
/// ones repeat the 28 s leg cycle (without the ramp), chained end to end.
/// The turns cancel (+/- kNavTurnRate for equal times), so every cycle
/// retraces the same S-bend shape translated along the original heading.
inline TypicalNavigationScenario typicalNavigation() {
  const std::vector<NavLeg> legs = {
      // Straight cruise before the first turn.
      {6.0, gtsam::Vector3::Zero(), gtsam::Vector3(kNavCruiseSpeed, 0, 0)},
      // Turn right (+z body rate is clockwise seen from above, as +z is down).
      {3.0, gtsam::Vector3(0, 0, kNavTurnRate),
       gtsam::Vector3(kNavCruiseSpeed, 0, 0)},
      // Straight cruise on the new heading.
      {7.0, gtsam::Vector3::Zero(), gtsam::Vector3(kNavCruiseSpeed, 0, 0)},
      // Turn left.
      {3.0, gtsam::Vector3(0, 0, -kNavTurnRate),
       gtsam::Vector3(kNavCruiseSpeed, 0, 0)},
      // Final straight cruise, held indefinitely beyond this leg.
      {9.0, gtsam::Vector3::Zero(), gtsam::Vector3(kNavCruiseSpeed, 0, 0)},
  };
  return TypicalNavigationScenario(kRampDuration, kNavCruiseSpeed, legs);
}

}  // namespace imu_scenarios

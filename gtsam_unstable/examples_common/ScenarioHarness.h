/**
 * @file  ScenarioHarness.h
 * @brief Shared run configuration, IMU/aiding simulation and summary reporting
 *        for the tg_eqf / tfg_inekf / mekf trajectory examples.
 *
 * Everything here is filter-agnostic, so the three example runners differ only
 * in how they build, propagate and update their own filter. Keeping the RNG
 * draw order and the error metrics in one place is what makes their CSVs
 * comparable run-for-run.
 */
#pragma once

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ScenarioRunner.h>

#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace imu_scenarios {

/// Gravity magnitude the examples integrate against (m/s^2).
inline constexpr double kGravity = 9.81;

/// Nav-frame gravity vector. The examples run in a Z-down (NED) navigation
/// frame: +z points down, so gravity is +kGravity on z and a vehicle's z
/// coordinate grows as it descends. This is the only place the examples commit
/// to a frame -- the IMU simulator and all three filters take gravity from
/// here, so the truth-side and filter-side gravity cannot disagree. Equivalent
/// to gtsam::PreintegrationParams::MakeSharedD(kGravity)->n_gravity.
inline Eigen::Vector3d navGravity() { return {0.0, 0.0, kGravity}; }

struct RunOptions {
  double duration = 30.0;
  double dt = 0.01;
  std::string output_path;

  // IMU error model. Defaults describe an ideal IMU (no bias, no noise) so the
  // clean dead-reckoning baseline is reproduced exactly. Per-channel (x, y, z)
  // rather than one isotropic value, like the init-*-sigma block below.
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();   // rad/s (initial)
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();  // m/s^2 (initial)
  Eigen::Vector3d gyro_noise_sigma =
      Eigen::Vector3d::Zero();  // rad/s/sqrt(Hz)
  Eigen::Vector3d accel_noise_sigma =
      Eigen::Vector3d::Zero();  // m/s^2/sqrt(Hz)
  Eigen::Vector3d gyro_bias_rw =
      Eigen::Vector3d::Zero();  // gyro bias random-walk rate (rad/s/sqrt(s))
  Eigen::Vector3d accel_bias_rw =
      Eigen::Vector3d::Zero();  // accel bias random-walk rate (m/s^2/sqrt(s))

  // Per-channel initial stddevs for the physical state, forming the
  // block-diagonal P0 and (via sampleInitialEstimate) the matching offset of
  // the filter's initial belief from the truth, so eps(0) ~ N(0, P0) holds.
  // Each block gets its own per-axis (x, y, z) stddev rather than one
  // isotropic value. States a filter estimates without a physical
  // counterpart (the TG-EqF virtual bias) are not covered here: the filter
  // initializes those itself.
  Eigen::Vector3d init_att_sigma =
      Eigen::Vector3d::Constant(0.1);  // initial attitude stddev (rad)
  Eigen::Vector3d init_vel_sigma =
      Eigen::Vector3d::Constant(0.1);  // initial velocity stddev (m/s)
  Eigen::Vector3d init_pos_sigma =
      Eigen::Vector3d::Constant(0.1);  // initial position stddev (m)
  Eigen::Vector3d init_gyro_bias_sigma =
      Eigen::Vector3d::Constant(0.1);  // initial gyro bias stddev
  Eigen::Vector3d init_accel_bias_sigma =
      Eigen::Vector3d::Constant(0.1);  // initial accel bias stddev

  unsigned seed = 42;  // RNG seed for noise (vary it for Monte Carlo)

  int log_decim = 1;  // log every Nth step (1 = every step)

  double pos_rate = 0.0;  // GNSS position update rate in Hz (0 = off)
  Eigen::Vector3d pos_noise_sigma =
      Eigen::Vector3d::Constant(0.1);  // position measurement noise stddev (m)

  double dvl_rate = 0.0;  // DVL body-velocity update rate in Hz (0 = off)
  Eigen::Vector3d dvl_noise_sigma = Eigen::Vector3d::Constant(
      0.02);  // DVL measurement noise stddev (m/s)

  double depth_rate = 0.0;  // pressure-sensor depth update rate in Hz (0 = off)
  double depth_noise_sigma = 0.05;  // depth measurement noise stddev (m)

  double aiding_start_time = 0.0;  // delay before aiding updates start (s)

  // TG-EqF only: the covariance transport (reset) step after each measurement
  // update, the depth output model -- equivariant pseudo-position (the
  // default) or the direct scalar one -- and the DVL output matrix: the
  // equivariant C* (default) or the naive chart derivative C0, for the
  // velocity-model ablation. Other filter adapters ignore all three.
  bool tg_eqf_reset_step = true;
  bool tg_eqf_depth_direct = false;
  bool tg_eqf_dvl_c0 = false;

  // Applies to all three filter adapters, unlike the two options above.
  // Default is the full upper triangle; pass --cov-format diag to opt out.
  bool log_full_covariance = true;
};

/// Parse "x,y,z" into a Vector3 (throws on malformed input).
inline Eigen::Vector3d parseVector3(const std::string& s) {
  Eigen::Vector3d v;
  std::size_t start = 0;
  for (int k = 0; k < 3; ++k) {
    const std::size_t comma = s.find(',', start);
    const std::string tok =
        s.substr(start, comma == std::string::npos ? comma : comma - start);
    v(k) = std::stod(tok);
    if (k < 2) {
      if (comma == std::string::npos) {
        throw std::runtime_error("expected 'x,y,z', got: " + s);
      }
      start = comma + 1;
    }
  }
  return v;
}

/// Parse the CLI shared by every example executable. Every recognized option
/// takes one value; a missing value throws rather than silently defaulting.
inline RunOptions parseRunOptions(int argc, char* argv[],
                                  const char* default_output) {
  RunOptions opts;
  opts.output_path = default_output;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("option " + arg + " requires a value");
      }
      return argv[++i];
    };

    if (arg == "--output") {
      opts.output_path = value();
    } else if (arg == "--duration") {
      opts.duration = std::stod(value());
    } else if (arg == "--dt") {
      opts.dt = std::stod(value());
    } else if (arg == "--gyro-bias") {
      opts.gyro_bias = parseVector3(value());
    } else if (arg == "--accel-bias") {
      opts.accel_bias = parseVector3(value());
    } else if (arg == "--gyro-noise") {
      opts.gyro_noise_sigma = parseVector3(value());
    } else if (arg == "--accel-noise") {
      opts.accel_noise_sigma = parseVector3(value());
    } else if (arg == "--gyro-bias-rw") {
      opts.gyro_bias_rw = parseVector3(value());
    } else if (arg == "--accel-bias-rw") {
      opts.accel_bias_rw = parseVector3(value());
    } else if (arg == "--init-att-sigma") {
      opts.init_att_sigma = parseVector3(value());
    } else if (arg == "--init-vel-sigma") {
      opts.init_vel_sigma = parseVector3(value());
    } else if (arg == "--init-pos-sigma") {
      opts.init_pos_sigma = parseVector3(value());
    } else if (arg == "--init-gyro-bias-sigma") {
      opts.init_gyro_bias_sigma = parseVector3(value());
    } else if (arg == "--init-accel-bias-sigma") {
      opts.init_accel_bias_sigma = parseVector3(value());
    } else if (arg == "--seed") {
      opts.seed = static_cast<unsigned>(std::stoul(value()));
    } else if (arg == "--log-decim") {
      opts.log_decim = std::max(1, std::stoi(value()));
    } else if (arg == "--pos-rate") {
      opts.pos_rate = std::stod(value());
    } else if (arg == "--pos-noise") {
      opts.pos_noise_sigma = parseVector3(value());
    } else if (arg == "--dvl-rate") {
      opts.dvl_rate = std::stod(value());
    } else if (arg == "--dvl-noise") {
      opts.dvl_noise_sigma = parseVector3(value());
    } else if (arg == "--depth-rate") {
      opts.depth_rate = std::stod(value());
    } else if (arg == "--depth-noise") {
      opts.depth_noise_sigma = std::stod(value());
    } else if (arg == "--aiding-start-time") {
      opts.aiding_start_time = std::stod(value());
    } else if (arg == "--reset-step") {
      const int flag = std::stoi(value());
      if (flag != 0 && flag != 1) {
        throw std::runtime_error("--reset-step requires 0 or 1");
      }
      opts.tg_eqf_reset_step = (flag != 0);
    } else if (arg == "--depth-model") {
      const std::string model = value();
      if (model == "pseudo") {
        opts.tg_eqf_depth_direct = false;
      } else if (model == "direct") {
        opts.tg_eqf_depth_direct = true;
      } else {
        throw std::runtime_error("--depth-model requires 'pseudo' or 'direct'");
      }
    } else if (arg == "--dvl-jacobian") {
      const std::string mode = value();
      if (mode == "cstar") {
        opts.tg_eqf_dvl_c0 = false;
      } else if (mode == "c0") {
        opts.tg_eqf_dvl_c0 = true;
      } else {
        throw std::runtime_error("--dvl-jacobian requires 'cstar' or 'c0'");
      }
    } else if (arg == "--cov-format") {
      const std::string format = value();
      if (format == "diag") {
        opts.log_full_covariance = false;
      } else if (format == "full") {
        opts.log_full_covariance = true;
      } else {
        throw std::runtime_error("--cov-format requires 'diag' or 'full'");
      }
    } else {
      // A stale binary once accepted --cov-format full, ignored it, and still
      // recorded the flag in the run manifest; silently ignoring an unknown
      // flag makes a stale binary indistinguishable from a working one.
      throw std::runtime_error("unrecognized argument '" + arg + "'");
    }
  }
  return opts;
}

/// Reject option combinations no runner can honour (throws).
inline void validateRunOptions(const RunOptions& opts) {
  if (opts.dt <= 0.0) {
    throw std::runtime_error("dt must be > 0");
  }
  if (opts.pos_rate > 0.0 && (opts.pos_noise_sigma.array() <= 0.0).any()) {
    throw std::runtime_error(
        "pos_noise_sigma must be > 0 on every axis when position updates are "
        "enabled (pos_rate > 0)");
  }
  if (opts.dvl_rate > 0.0 && (opts.dvl_noise_sigma.array() <= 0.0).any()) {
    throw std::runtime_error(
        "dvl_noise_sigma must be > 0 on every axis when DVL updates are "
        "enabled (dvl_rate > 0)");
  }
  if (opts.depth_rate > 0.0 && opts.depth_noise_sigma <= 0.0) {
    throw std::runtime_error(
        "depth_noise_sigma must be > 0 when depth updates are enabled "
        "(depth_rate > 0)");
  }
}

/// Covariance of the physical state every filter shares, ordered
/// [attitude, velocity, position, gyro bias, accel bias].
using PhysicalStateCovariance = Eigen::Matrix<double, 15, 15>;

/// Block-diagonal P0 = diag(sigma^2) over the physical state, matching the
/// offset sampleInitialEstimate() draws. A filter that also estimates states
/// with no physical counterpart (the TG-EqF virtual bias) extends this itself;
/// none of the init-*-sigma options describe those.
inline PhysicalStateCovariance initialCovariance(const RunOptions& opts) {
  Eigen::Matrix<double, 15, 1> sd;
  sd.segment<3>(0) = opts.init_att_sigma;
  sd.segment<3>(3) = opts.init_vel_sigma;
  sd.segment<3>(6) = opts.init_pos_sigma;
  sd.segment<3>(9) = opts.init_gyro_bias_sigma;
  sd.segment<3>(12) = opts.init_accel_bias_sigma;
  return sd.array().square().matrix().asDiagonal();
}

/// The belief a filter is initialized with: the ground truth at t = 0 offset by
/// a draw from the same sigmas that build P0, so eps(0) ~ N(0, P0) holds. The
/// filter never sees the truth itself.
struct InitialEstimate {
  gtsam::NavState nav;
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
};

/// Draw one InitialEstimate around the clean initial truth, consuming `rng`.
/// Attitude gets a world-frame rotation offset; velocity, position and the two
/// biases get additive offsets, the biases around their true initial values.
template <typename Rng>
inline InitialEstimate sampleInitialEstimate(Rng& rng, const RunOptions& opts,
                                             const gtsam::NavState& truth) {
  std::normal_distribution<double> normal(0.0, 1.0);
  auto draw3 = [&](const Eigen::Vector3d& sigma) {
    return Eigen::Vector3d(sigma.x() * normal(rng), sigma.y() * normal(rng),
                           sigma.z() * normal(rng));
  };
  const gtsam::Rot3 dR = gtsam::Rot3::Expmap(draw3(opts.init_att_sigma));
  const Eigen::Vector3d dv = draw3(opts.init_vel_sigma);
  const Eigen::Vector3d dp = draw3(opts.init_pos_sigma);

  InitialEstimate initial;
  initial.nav = gtsam::NavState(dR * truth.attitude(), truth.position() + dp,
                                truth.velocity() + dv);
  initial.bias_gyro = draw3(opts.init_gyro_bias_sigma);
  initial.bias_accel = draw3(opts.init_accel_bias_sigma);
  return initial;
}

/// Fires at a fixed rate, first at `start_time + 1/rate`. A non-positive rate
/// never fires.
class PeriodicTrigger {
 public:
  PeriodicTrigger(double rate, double start_time)
      : period_(rate > 0.0 ? 1.0 / rate : 0.0),
        next_(rate > 0.0 ? start_time + period_
                         : std::numeric_limits<double>::infinity()) {}

  /// True once per period; advances the schedule when it fires.
  bool due(double t) {
    if (t < next_ - 1e-9) {
      return false;
    }
    next_ += period_;
    return true;
  }

 private:
  double period_;
  double next_;
};

/// Noisy GNSS-style position measurement, world frame.
struct PositionMeasurement {
  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;
};

/// Noisy DVL measurement of the body-frame velocity h(xi) = R^T v.
struct DvlMeasurement {
  Eigen::Vector3d body_velocity;
  Eigen::Matrix3d covariance;
};

/// Noisy pressure-sensor measurement of the world-frame z position, which in
/// the examples' Z-down (NED) frame is depth below the surface (positive down).
struct DepthMeasurement {
  double depth;
  Eigen::Matrix<double, 1, 1> covariance;
};

/// IMU signals handed to a filter: corrupted by bias and white noise.
struct ImuMeasurement {
  Eigen::Vector3d omega;
  Eigen::Vector3d accel;
};

/// Corrupts the scenario's true signals into sensor readings and random-walks
/// the true IMU biases. Owns the draw order (gyro, accel, then aiding noise,
/// then the bias walk) so a given seed produces the same sensor stream for
/// every filter. The RNG is borrowed, not owned: the caller draws the initial
/// perturbation from it first.
class ImuSimulator {
 public:
  ImuSimulator(const RunOptions& opts, std::mt19937& rng,
               const Eigen::Vector3d& gyro_bias,
               const Eigen::Vector3d& accel_bias)
      : rng_(rng),
        gyro_bias_(gyro_bias),
        accel_bias_(accel_bias),
        gyro_sd_(opts.gyro_noise_sigma / std::sqrt(opts.dt)),
        accel_sd_(opts.accel_noise_sigma / std::sqrt(opts.dt)),
        gyro_rw_step_(opts.gyro_bias_rw * std::sqrt(opts.dt)),
        accel_rw_step_(opts.accel_bias_rw * std::sqrt(opts.dt)) {}

  /// IMU reading at time t: truth + current true bias + white noise.
  ImuMeasurement sample(const gtsam::ScenarioRunner& runner, double t) {
    ImuMeasurement imu;
    imu.omega = runner.actualAngularVelocity(t) + gyro_bias_ + draw(gyro_sd_);
    imu.accel = runner.actualSpecificForce(t) + accel_bias_ + draw(accel_sd_);
    return imu;
  }

  /// GNSS-style position measurement in the world frame.
  PositionMeasurement samplePosition(const gtsam::NavState& gt,
                                     const Eigen::Vector3d& sigma) {
    return {gt.position() + draw(sigma), diagonal(sigma)};
  }

  /// DVL measurement of the body-frame velocity, h(xi) = R^T v.
  DvlMeasurement sampleDvl(const gtsam::NavState& gt,
                           const Eigen::Vector3d& sigma) {
    const Eigen::Vector3d body_vel = gt.attitude().unrotate(gt.velocity());
    return {body_vel + draw(sigma), diagonal(sigma)};
  }

  /// Pressure-sensor measurement of the world-frame z position -- depth below
  /// the surface, since the examples' nav frame is Z-down. Draws a single
  /// normal, since the sensor reports one channel.
  DepthMeasurement sampleDepth(const gtsam::NavState& gt, double sigma) {
    Eigen::Matrix<double, 1, 1> covariance;
    covariance(0, 0) = sigma * sigma;
    return {gt.position().z() + sigma * normal_(rng_), covariance};
  }

  /// Advance the true biases by one random-walk step (no-op at zero rate).
  void randomWalkBias() {
    if (gyro_rw_step_.squaredNorm() > 0.0) {
      gyro_bias_ += draw(gyro_rw_step_);
    }
    if (accel_rw_step_.squaredNorm() > 0.0) {
      accel_bias_ += draw(accel_rw_step_);
    }
  }

  const Eigen::Vector3d& gyroBias() const { return gyro_bias_; }
  const Eigen::Vector3d& accelBias() const { return accel_bias_; }

 private:
  Eigen::Vector3d draw(const Eigen::Vector3d& sd) {
    return Eigen::Vector3d(sd.x() * normal_(rng_), sd.y() * normal_(rng_),
                           sd.z() * normal_(rng_));
  }
  static Eigen::Matrix3d diagonal(const Eigen::Vector3d& sigma) {
    return sigma.array().square().matrix().asDiagonal();
  }

  std::mt19937& rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  Eigen::Vector3d gyro_bias_;
  Eigen::Vector3d accel_bias_;
  Eigen::Vector3d gyro_sd_;
  Eigen::Vector3d accel_sd_;
  Eigen::Vector3d gyro_rw_step_;
  Eigen::Vector3d accel_rw_step_;
};

struct RunSummary {
  size_t num_samples = 0;
  double final_att_error = 0.0;
  double final_pos_error = 0.0;
  double final_vel_error = 0.0;
  double rms_att_error = 0.0;
  double rms_pos_error = 0.0;
  double rms_vel_error = 0.0;
};

/// Accumulates the physical (chart-independent) estimate errors so the three
/// filters report the same quantity: world-frame position/velocity error and
/// the geodesic attitude error.
class RunSummaryAccumulator {
 public:
  void add(const gtsam::NavState& gt, const gtsam::Rot3& est_R,
           const Eigen::Vector3d& est_v, const Eigen::Vector3d& est_p) {
    final_pos_ = (Eigen::Vector3d(gt.position()) - est_p).norm();
    final_vel_ = (Eigen::Vector3d(gt.velocity()) - est_v).norm();
    final_att_ = gtsam::Rot3::Logmap(gt.attitude().between(est_R)).norm();

    sum_pos_sq_ += final_pos_ * final_pos_;
    sum_vel_sq_ += final_vel_ * final_vel_;
    sum_att_sq_ += final_att_ * final_att_;
    ++num_samples_;
  }

  RunSummary summary() const {
    RunSummary s;
    s.num_samples = num_samples_;
    s.final_pos_error = final_pos_;
    s.final_vel_error = final_vel_;
    s.final_att_error = final_att_;
    if (num_samples_ > 0) {
      const auto n = static_cast<double>(num_samples_);
      s.rms_pos_error = std::sqrt(sum_pos_sq_ / n);
      s.rms_vel_error = std::sqrt(sum_vel_sq_ / n);
      s.rms_att_error = std::sqrt(sum_att_sq_ / n);
    }
    return s;
  }

 private:
  size_t num_samples_ = 0;
  double final_pos_ = 0.0, final_vel_ = 0.0, final_att_ = 0.0;
  double sum_pos_sq_ = 0.0, sum_vel_sq_ = 0.0, sum_att_sq_ = 0.0;
};

/// Standard console footer for a completed run.
inline void printRunSummary(const RunSummary& summary,
                            const std::string& scenario_name,
                            const std::string& output_path) {
  std::cout << scenario_name << " scenario complete.\n";
  std::cout << "  output: " << output_path << '\n';
  std::cout << "  samples: " << summary.num_samples << '\n';
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "  final position error (m): " << summary.final_pos_error
            << '\n';
  std::cout << "  final velocity error (m/s): " << summary.final_vel_error
            << '\n';
  std::cout << "  final attitude error (rad): " << summary.final_att_error
            << '\n';
  std::cout << "  RMS position error (m): " << summary.rms_pos_error << '\n';
  std::cout << "  RMS velocity error (m/s): " << summary.rms_vel_error << '\n';
  std::cout << "  RMS attitude error (rad): " << summary.rms_att_error << '\n';
}

}  // namespace imu_scenarios

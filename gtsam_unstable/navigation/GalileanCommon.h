#pragma once 

#include <gtsam/base/Matrix.h>
#include <gtsam/base/ProductLieGroup.h> // Matrix3, Matrix
#include <gtsam/base/TangentLieGroup.h> // Group G it's Lie Algebra
#include <gtsam/base/Vector.h> // Vector types vector3
#include <gtsam/base/VectorConstants.h> // constraints like zero vectors
#include <gtsam/geometry/Gal3.h>  // This is the Galilean Group itself
#include <gtsam_unstable/dllexport.h> // marks classes so it can be used outside the compiled library

#include <deque> // (just for th ebuffer)

namespace gtsam {
namespace galilean {

using Vector10 = Gal3::TangentVector; // (w, w, w, v, v v, p, p, p t) represenmtation
using Matrix10 = Gal3::Jacobian; // (10 x 10 Jacobian)

using Vector20 = Eigen::Matrix<double, 20, 1>; // 20 vectors
using Matrix20 = Eigen::Matrix<double, 20, 20>; // 20 x 20 matricies
// 10 galilean frame, 10 biases

// GNSS jacobians (GNSS) - position, 3 rows 10 columns eq.40
using Matrix3x10 = Eigen::Matrix<double, 3, 10>;
using Matrix3x20 = Eigen::Matrix<double, 3, 20>;

// Eq.26 G = Gal(3) ⋉ gal(3) (A, a)
using Group = TangentLieGroup<Gal3>; // A  Gal 3 , galilean transormation, a Vector 10

/// State manifold M = Gal(3) × ℝ¹⁰, ξ = (F, b) (eq. 24).
/// F is the cross-body Galilean frame (time component = delay δ) and b holds
/// the gyro, accelerometer and two virtual biases in gal(3) coordinates.
using State = ProductLieGroup<Gal3, Vector10>; // physical state 

/// Dimension of the state manifold and of the error coordinates.
inline constexpr int kStateDim = 20;

// Index b
// locks of the 20-dim error coordinates (and of the covariance).
namespace index {
inline constexpr int kAttitude = 0;    ///< 3: attitude 0, 1, 2 rotation
inline constexpr int kVelocity = 3;    ///< 3: velocity 3, 4, 5 velocity error
inline constexpr int kPosition = 6;    ///< 3: position 6, 7, 8 position error
inline constexpr int kDelay = 9;       ///< 1: GNSS delay δ
inline constexpr int kGyroBias = 10;   ///< 3: gyroscope bias
inline constexpr int kAccelBias = 13;  ///< 3: accelerometer bias
                                       ///
inline constexpr int kVirtualBias = 16;  ///< 4: virtual biases (ν, ρ)
                                         ///
}  // namespace index

/// One IMU sample.
struct GTSAM_UNSTABLE_EXPORT IMUInput {

  double stamp = -1.0;  ///< Optional timestamp [s].
                      
  Vector3 gyr = Vector3::Zero();  ///< Angular rate [rad/s]. gyroscope
  Vector3 acc = Vector3::Zero();  ///< Specific force [m/s²]. accelerometer

  IMUInput() = default; // everything set to zero

  IMUInput(const Vector3& gyr_, const Vector3& acc_, double stamp_ = -1.0) : stamp(stamp_), gyr(gyr_), acc(acc_) {}

  /// Extended inertial input w_N = (ω, a, 0, 1) ∈ ℝ¹⁰ (eq. 24).
  Vector10 extended() const {Vector10 w;w << gyr, acc, Vector3::Zero(), 1.0;return w;}
};

/// A GNSS position fix, reported at arrival time but taken at t - δ.
struct GTSAM_UNSTABLE_EXPORT GnssMeasurement {
  double stamp = -1.0;                         ///< Optional arrival time [s].
  Point3 position = Point3::Zero();            ///< Antenna position [m].
  Matrix3 covariance = Matrix3::Identity();    ///< Position covariance [m²].

  GnssMeasurement() = default;
  GnssMeasurement(const Point3& position_, const Matrix3& covariance_,
                  double stamp_ = -1.0)
      : stamp(stamp_), position(position_), covariance(covariance_) {}
};

/// Constant Earth input g_N = (ω_E, -g, 0, 1) ∈ ℝ¹⁰ (eq. 24).
/// `gravity` is the gravity vector in the Earth frame, e.g. (0, 0, -9.81).
inline Vector10 EarthInput(const Vector3& gravity, const Vector3& earthRate) {
  Vector10 g;
  g << earthRate, -gravity, Vector3::Zero(), 1.0;
  return g;
}

/// @name Readable accessors for the state ξ = (F, b)
/// @{
inline const Gal3& galileanFrame(const State& xi) { return xi.first; }
inline const Vector10& bias(const State& xi) { return xi.second; }
inline Vector3 gyroBias(const State& xi) { return xi.second.head<3>(); }
inline Vector3 accelBias(const State& xi) { return xi.second.segment<3>(3); }
/// @}

/**
 * Sliding buffer of IMU preintegration matrices Υ (Sec. 2.4, eq. 20).
 *
 * Entry k holds Υ_k = ∏_{s_k}^{now} exp(w̃^ dt) together with the input w̃_k
 * held on [s_k, s_k + dt_k). Υ(δ) for any δ in the window is recovered
 * exactly (under zero-order hold) by trimming the first held interval.
 */
class GTSAM_UNSTABLE_EXPORT UpsilonBuffer {

 public:

  explicit UpsilonBuffer(double window = 0.6) : window_(window) {}

  /// Append input w̃ = w_N - b̂ held for dt and propagate all entries (eq. 20).
  void push(const Vector10& wTilde, double dt);

  /**
   * Preintegration matrix Υ(δ) from t-δ to now.
   * @param delta Requested delay, clamped to [0, available window].
   * @param wStart Optional: the input w̃ held at time t-δ, needed for ∂Υ/∂δ.
   */
  Gal3 at(double delta, Vector10* wStart = nullptr) const;

  /// Length of history currently available, in seconds.
  double span() const;
  bool empty() const { return entries_.empty(); }
  size_t size() const { return entries_.size(); }
  double window() const { return window_; }

 private:

  struct Entry {
    double start;    ///< s_k
    Vector10 input;  ///< w̃ held on [s_k, s_k + dt_k)
    Gal3 upsilon;    ///< ∏ from s_k to now
  };

  double window_;
  double now_ = 0.0;
  std::deque<Entry> entries_;
};

}  // namespace galilean
}  // namespace gtsam

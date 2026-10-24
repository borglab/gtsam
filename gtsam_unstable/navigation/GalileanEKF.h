#pragma once

#include <gtsam/navigation/EquivariantFilter.h>
#include <gtsam_unstable/navigation/GalileanSymmetry.h>

#include <limits>

namespace gtsam {
namespace galilean {

/**
 * @brief Runtime parameters for the Galilean filter.
 *
 * Noise densities are continuous-time (per √Hz); variances are per-axis
 * scalars applied to identity blocks, as in EqVIOFilterParams.
 */
struct GTSAM_UNSTABLE_EXPORT GalileanFilterParams {
  /// @name Physical model
  /// @{
  /// Gravity vector in the Earth (navigation) frame. ENU default; NED is
  /// (0, 0, +9.81).
  Vector3 gravity = Vector3(0, 0, -9.81);
  /// Earth rotation rate ω_E in the Earth frame (0 to neglect).
  Vector3 earthRate = Vector3::Zero();
  /// GNSS antenna position p₀ in the IMU frame (eq. 37).
  Point3 antennaLever = Point3::Zero();
  /// Length of the Υ buffer [s]; must exceed the largest delay.
  double bufferWindow = 0.6;
  /// @}

  /// @name Initial state used by initializeFromIMU()
  /// @{
  double initialDelay = 0.0;            ///< δ̂₀ [s]
  double initialAttitudeSigma = 0.1;    ///< [rad]
  double initialVelocitySigma = 1.0;    ///< [m/s]
  double initialPositionSigma = 10.0;   ///< [m]
  double initialDelaySigma = 0.2;       ///< [s]
  double initialGyroBiasSigma = 1e-2;   ///< [rad/s]
  double initialAccelBiasSigma = 1e-1;  ///< [m/s²]
  double initialVirtualBiasSigma = 1e-4;
  /// @}

  /// @name IMU input noise (continuous time)
  /// @{
  double gyroNoiseDensity = 1e-3;       ///< [rad/s/√Hz]
  double accelNoiseDensity = 1e-2;      ///< [m/s²/√Hz]
  double gyroBiasRandomWalk = 1e-5;     ///< [rad/s²/√Hz]
  double accelBiasRandomWalk = 1e-4;    ///< [m/s³/√Hz]
  double virtualInputNoise = 1e-5;      ///< ν and ρ virtual inputs
  double virtualBiasRandomWalk = 1e-6;  ///< virtual biases
  /// @}

  /// @name Additional state process noise (variance per second)
  /// @{
  double attitudeProcessVariance = 0.0;
  double velocityProcessVariance = 0.0;
  double positionProcessVariance = 0.0;
  /// Random walk on δ, for time-varying latency [s²/s].
  double delayProcessVariance = 0.0;
  /// @}

  /// @name Update
  /// @{
  /// Apply the covariance reset after each update (Alg. 1, ⋆⋆).
  bool resetStep = true;
  /// NIS gate (χ², 3 dof). 16.27 rejects at the 99.9% level; ∞ disables.
  double gnssOutlierThreshold = std::numeric_limits<double>::infinity();
  /// Number of accepted GNSS updates before the NIS gate is enabled.
  int gnssGateWarmup = 20;
  /// After this many consecutive rejections the next fix is accepted anyway:
  /// a long run of "outliers" means the filter, not the GNSS, is wrong, and a
  /// gate that never reopens would lock the filter out permanently.
  int gnssMaxConsecutiveRejections = 5;
  /// @}

  /// Continuous-time input noise Q (20x20), ordered
  /// [gyro, acc, ν virtual, ρ virtual, gyro bias, acc bias, virtual biases].
  Matrix20 inputNoise() const;
  /// Additional state process noise (20x20, variance per second).
  Matrix20 stateProcessNoise() const;
  /// Initial covariance in error coordinates from the initial sigmas.
  Matrix20 initialCovariance() const;
};

/**
 * @brief Galilean equivariant filter built on top of `EquivariantFilter`.
 *
 * Jointly estimates attitude, velocity, position, the unknown GNSS delay δ
 * and IMU biases. Propagation passes the exact discrete lift to
 * `predictWithStep(...)`; the Υ buffer and output model handle the delay.
 */
class GTSAM_UNSTABLE_EXPORT GalileanFilter
    : public EquivariantFilter<State, Symmetry> {
 public:
  using Base = EquivariantFilter<State, Symmetry>;
  using Params = GalileanFilterParams;

  /// Construct uninitialized; call initializeFromIMU() before use.
  explicit GalileanFilter(const Params& params);

  /**
   * Construct initialized from a known navigation state.
   * @param params Filter parameters.
   * @param navState Initial isochronous navigation state T(t) (time = 0).
   * @param delay Initial delay guess δ̂₀.
   * @param bias Initial bias b̂₀ in gal(3) coordinates.
   * @param P0 Initial covariance in error coordinates (20x20).
   */
  GalileanFilter(const Params& params, const Gal3& navState, double delay,
                 const Vector10& bias, const Matrix20& P0);

  /**
   * @brief Initialize attitude from gravity in one IMU sample.
   *
   * The measured specific force is treated as -gravity (vehicle at rest) and
   * aligned with the Earth "up" direction. Position, velocity and biases start
   * at zero with the initial sigmas from the parameters.
   */
  void initializeFromIMU(const IMUInput& imu);

  /**
   * @brief Initialize from a known (rough) navigation state.
   *
   * Use when the vehicle is already moving, e.g. with position from the first
   * GNSS fix. Delay and covariance come from the parameters.
   * @param navState Initial isochronous navigation state T(t) (time = 0).
   * @param bias Initial bias b̂₀ in gal(3) coordinates.
   */
  void initialize(const Gal3& navState,
                  const Vector10& bias = Vector10::Zero());

  /// Propagate across one IMU hold interval (Alg. 1, propagation).
  /// No-op before initialization or for dt <= 0.
  void predict(const IMUInput& imu, double dt);

  /**
   * @brief Fuse a delayed GNSS position (Alg. 1, update).
   * @return false if not initialized or rejected by the outlier gate.
   * @throws std::invalid_argument if the covariance is not positive definite.
   */
  bool update(const GnssMeasurement& measurement);

  /// @name Estimates
  /// @{
  bool isInitialized() const { return initialized_; }
  /// Current navigation state T̂(t) = Γ(δ̂)⁻¹ F̂ (eq. 23), time = 0.
  Gal3 navState() const;
  /// Navigation state at the delayed time, T̂(t-δ̂) = F̂ Υ(δ̂)⁻¹ (eq. 17).
  Gal3 delayedNavState() const;
  Rot3 attitude() const { return navState().rotation(); }
  Vector3 velocity() const { return navState().velocity(); }
  Point3 position() const { return navState().translation(); }
  /// Estimated GNSS delay δ̂ [s].
  double delay() const { return galileanFrame(state()).time(); }
  double delaySigma() const;
  Vector3 gyroBias() const { return galilean::gyroBias(state()); }
  Vector3 accelBias() const { return galilean::accelBias(state()); }
  /// Normalized innovation squared of the last GNSS update.
  double lastNis() const { return lastNis_; }
  /// Number of GNSS updates accepted since initialization.
  int acceptedUpdates() const { return acceptedUpdates_; }
  /// Number of GNSS updates rejected in a row by the gate.
  int consecutiveRejections() const { return consecutiveRejections_; }
  const Params& params() const { return params_; }
  const UpsilonBuffer& buffer() const { return buffer_; }
  /// @}

  /// @name Linearization (public for testing)
  /// @{
  /// Earth-frame intra-body transform Γ(δ) = exp(g_N^ δ).
  Gal3 Gamma(double delta) const;
  /// GNSS prediction h(ξ) with Jacobian w.r.t. the state chart at ξ.
  Point3 predictGnss(const State& xi, OptionalJacobian<3, 20> H = {}) const;
  /// Output matrix C in error coordinates at the current estimate.
  Matrix3x20 measurementMatrixC() const;
  /// @}

 private:
  /// Group element X₀ with φ(ξ₀, X₀) = (Γ(δ) T, b).
  static Group InitialGroup(const Params& params, const Gal3& navState,
                            double delay, const Vector10& bias);

  Params params_;
  Vector10 gN_;
  UpsilonBuffer buffer_;
  bool initialized_ = false;
  double lastNis_ = 0.0;
  int acceptedUpdates_ = 0;
  int consecutiveRejections_ = 0;
};

}  // namespace galilean
}  // namespace gtsam

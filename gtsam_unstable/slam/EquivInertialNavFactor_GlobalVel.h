
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   EquivInertialNavFactor_GlobalVel.h
 *  @author Vadim Indelman, Stephen Williams
 *  @brief  Equivalent inertial navigation factor (velocity in the global frame).
 *  @date   Sep. 26, 2012
 **/

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>

// Using numerical derivative to calculate d(Pose3::Expmap)/dw
#include <gtsam/base/numericalDerivative.h>

#include <boost/bind/bind.hpp>
#include <boost/optional.hpp>

#include <ostream>

namespace gtsam {

/*
 * NOTES:
 * =====
 * Concept: Based on [Lupton12tro]
 * - Pre-integrate IMU measurements using the static function PreIntegrateIMUObservations.
 *    Pre-integrated quantities are expressed in the body system of t0 - the first time instant (in which pre-integration began).
 *    All sensor-to-body transformations are performed here.
 * - If required, calculate inertial solution by calling the static functions: predictPose_inertial, predictVelocity_inertial.
 * - When the time is right, incorporate pre-integrated IMU data by creating an EquivInertialNavFactor_GlobalVel factor, which will
 *   relate between navigation variables at the two time instances (t0 and current time).
 *
 * Other notes:
 * - The global frame (NED or ENU) is defined by the user by specifying the gravity vector in this frame.
 * - The IMU frame is implicitly defined by the user via the rotation matrix between global and imu frames.
 * - Camera and IMU frames are identical
 * - The user should specify a continuous equivalent noise covariance, which can be calculated using
 *   the static function CalcEquivalentNoiseCov based on the IMU gyro and acc measurement noise covariance
 *   matrices and the process\modeling covariance matrix. The IneritalNavFactor converts this into a
 *   discrete form using the supplied delta_t between sub-sequential measurements.
 * - Earth-rate correction:
 *     + Currently the user should supply R_ECEF_to_G, which is the rotation from ECEF to the global
 *       frame (Local-Level system: ENU or NED, see above).
 *     + R_ECEF_to_G can be calculated by approximated values of latitude and longitude of the system.
 *    + Currently it is assumed that a relatively small distance is traveled w.r.t. to initial pose, since R_ECEF_to_G is constant.
 *      Otherwise, R_ECEF_to_G should be updated each time using the current lat-lon.
 *
 * - Frame Notation:
 *   Quantities are written as {Frame of Representation/Destination Frame}_{Quantity Type}_{Quatity Description/Origination Frame}
 *   So, the rotational velocity of the sensor written in the body frame is: body_omega_sensor
 *   And the transformation from the body frame to the world frame would be: world_P_body
 *   This allows visual chaining. For example, converting the sensed angular velocity of the IMU
 *   (angular velocity of the sensor in the sensor frame) into the world frame can be performed as:
 *       world_R_body * body_R_sensor * sensor_omega_sensor = world_omega_sensor
 *
 *
 * - Common Quantity Types
 *   P : pose/3d transformation
 *   R : rotation
 *   omega : angular velocity
 *   t : translation
 *   v : velocity
 *   a : acceleration
 *
 * - Common Frames
 *   sensor : the coordinate system attached to the sensor origin
 *   body   : the coordinate system attached to body/inertial frame.
 *            Unless an optional frame transformation is provided, the
 *            sensor frame and the body frame will be identical
 *   world  : the global/world coordinate frame. This is assumed to be
 *            a tangent plane to the earth's surface somewhere near the
 *            vehicle
 */

template<class POSE, class VELOCITY, class IMUBIAS>
class EquivInertialNavFactor_GlobalVel : public NoiseModelFactor5<POSE, VELOCITY, IMUBIAS, POSE, VELOCITY> {

private:

  typedef EquivInertialNavFactor_GlobalVel<POSE, VELOCITY, IMUBIAS> This;
  typedef NoiseModelFactor5<POSE, VELOCITY, IMUBIAS, POSE, VELOCITY> Base;

  Vector delta_pos_in_t0_;
  Vector delta_vel_in_t0_;
  Vector3 delta_angles_;
  double dt12_;

  Vector world_g_;
  Vector world_rho_;
  Vector world_omega_earth_;

  Matrix Jacobian_wrt_t0_Overall_;

  boost::optional<IMUBIAS> Bias_initial_; // Bias used when pre-integrating IMU measurements
  boost::optional<POSE> body_P_sensor_;   // The pose of the sensor in the body frame

public:

  // shorthand for a smart pointer to a factor
  typedef typename boost::shared_ptr<EquivInertialNavFactor_GlobalVel> shared_ptr;

  /** default constructor - only use for serialization */
  EquivInertialNavFactor_GlobalVel() {}

  /** Constructor */
  EquivInertialNavFactor_GlobalVel(const Key& Pose1, const Key& Vel1, const Key& IMUBias1, const Key& Pose2, const Key& Vel2,
      const Vector& delta_pos_in_t0, const Vector& delta_vel_in_t0, const Vector3& delta_angles,
      double dt12, const Vector world_g, const Vector world_rho,
      const Vector& world_omega_earth, const noiseModel::Gaussian::shared_ptr& model_equivalent,
      const Matrix& Jacobian_wrt_t0_Overall,
      boost::optional<IMUBIAS> Bias_initial = boost::none, boost::optional<POSE> body_P_sensor = boost::none) :
        Base(model_equivalent, Pose1, Vel1, IMUBias1, Pose2, Vel2),
        delta_pos_in_t0_(delta_pos_in_t0), delta_vel_in_t0_(delta_vel_in_t0), delta_angles_(delta_angles),
        dt12_(dt12), world_g_(world_g), world_rho_(world_rho), world_omega_earth_(world_omega_earth), Jacobian_wrt_t0_Overall_(Jacobian_wrt_t0_Overall),
        Bias_initial_(Bias_initial), body_P_sensor_(body_P_sensor) {  }

  ~EquivInertialNavFactor_GlobalVel() override {}

  /** implement functions needed for Testable */

  /** print */
  void print(const std::string& s = "EquivInertialNavFactor_GlobalVel", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ","
        << keyFormatter(this->key3()) << ","
        << keyFormatter(this->key4()) << ","
        << keyFormatter(this->key5()) << "\n";
    std::cout << "delta_pos_in_t0: " << this->delta_pos_in_t0_.transpose() << std::endl;
    std::cout << "delta_vel_in_t0: " << this->delta_vel_in_t0_.transpose() << std::endl;
    std::cout << "delta_angles: " << this->delta_angles_ << std::endl;
    std::cout << "dt12: " << this->dt12_ << std::endl;
    std::cout << "gravity (in world frame): " << this->world_g_.transpose() << std::endl;
    std::cout << "craft rate (in world frame): " << this->world_rho_.transpose() << std::endl;
    std::cout << "earth's rotation (in world frame): " << this->world_omega_earth_.transpose() << std::endl;
    if(this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
    this->noiseModel_->print("  noise model");
  }

  /** equals */
  bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != nullptr && Base::equals(*e, tol)
    && (delta_pos_in_t0_ - e->delta_pos_in_t0_).norm() < tol
    && (delta_vel_in_t0_ - e->delta_vel_in_t0_).norm() < tol
    && (delta_angles_ - e->delta_angles_).norm() < tol
    && (dt12_ - e->dt12_) < tol
    && (world_g_ - e->world_g_).norm() < tol
    && (world_rho_ - e->world_rho_).norm() < tol
    && (world_omega_earth_ - e->world_omega_earth_).norm() < tol
    && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
  }


  POSE predictPose(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1) const {

    // Correct delta_pos_in_t0_ using (Bias1 - Bias_t0)
    Vector delta_BiasAcc  = Bias1.accelerometer();
    Vector delta_BiasGyro = Bias1.gyroscope();
    if (Bias_initial_){
      delta_BiasAcc  -= Bias_initial_->accelerometer();
      delta_BiasGyro -= Bias_initial_->gyroscope();
    }

    Matrix J_Pos_wrt_BiasAcc  = Jacobian_wrt_t0_Overall_.block(4,9,3,3);
    Matrix J_Pos_wrt_BiasGyro = Jacobian_wrt_t0_Overall_.block(4,12,3,3);
    Matrix J_angles_wrt_BiasGyro = Jacobian_wrt_t0_Overall_.block(0,12,3,3);

    /* Position term */
    Vector delta_pos_in_t0_corrected = delta_pos_in_t0_ + J_Pos_wrt_BiasAcc*delta_BiasAcc + J_Pos_wrt_BiasGyro*delta_BiasGyro;

    /* Rotation term */
    Vector delta_angles_corrected = delta_angles_ + J_angles_wrt_BiasGyro*delta_BiasGyro;
    // Another alternative:
    //    Vector delta_angles_corrected = Rot3::Logmap( Rot3::Expmap(delta_angles_)*Rot3::Expmap(J_angles_wrt_BiasGyro*delta_BiasGyro) );

    return predictPose_inertial(Pose1, Vel1,
        delta_pos_in_t0_corrected, delta_angles_corrected,
          dt12_, world_g_, world_rho_, world_omega_earth_);
  }

  static inline POSE predictPose_inertial(const POSE& Pose1, const VELOCITY& Vel1,
      const Vector& delta_pos_in_t0, const Vector3& delta_angles,
      const double dt12, const Vector& world_g, const Vector& world_rho, const Vector& world_omega_earth){

    const POSE& world_P1_body = Pose1;
    const VELOCITY& world_V1_body = Vel1;

    /* Position term */
    Vector body_deltaPos_body = delta_pos_in_t0;

    Vector world_deltaPos_pls_body = world_P1_body.rotation().matrix() * body_deltaPos_body;
    Vector world_deltaPos_body     = world_V1_body * dt12 + 0.5*world_g*dt12*dt12 + world_deltaPos_pls_body;

    // Incorporate earth-related terms. Note - these are assumed to be constant between t1 and t2.
    world_deltaPos_body -= 2*skewSymmetric(world_rho + world_omega_earth)*world_V1_body * dt12*dt12;

    /* TODO: the term dt12*dt12 in 0.5*world_g*dt12*dt12 is not entirely correct:
     *  the gravity should be canceled from the accelerometer measurements, bust since position
     *  is added with a delta velocity from a previous term, the actual delta time is more complicated.
     *  Need to figure out this in the future - currently because of this issue we'll get some more error
     *  in Z axis.
     */

    /* Rotation term */
    Vector body_deltaAngles_body = delta_angles;

    // Convert earth-related terms into the body frame
    Matrix body_R_world(world_P1_body.rotation().inverse().matrix());
    Vector body_rho = body_R_world * world_rho;
    Vector body_omega_earth = body_R_world * world_omega_earth;

    // Incorporate earth-related terms. Note - these are assumed to be constant between t1 and t2.
    body_deltaAngles_body -= (body_rho + body_omega_earth)*dt12;

    return POSE(Pose1.rotation() * POSE::Rotation::Expmap(body_deltaAngles_body), Pose1.translation() + typename POSE::Translation(world_deltaPos_body));

  }

  VELOCITY predictVelocity(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1) const {

    // Correct delta_vel_in_t0_ using (Bias1 - Bias_t0)
    Vector delta_BiasAcc  = Bias1.accelerometer();
    Vector delta_BiasGyro = Bias1.gyroscope();
    if (Bias_initial_){
      delta_BiasAcc  -= Bias_initial_->accelerometer();
      delta_BiasGyro -= Bias_initial_->gyroscope();
    }

    Matrix J_Vel_wrt_BiasAcc  = Jacobian_wrt_t0_Overall_.block(6,9,3,3);
    Matrix J_Vel_wrt_BiasGyro = Jacobian_wrt_t0_Overall_.block(6,12,3,3);

    Vector delta_vel_in_t0_corrected = delta_vel_in_t0_ + J_Vel_wrt_BiasAcc*delta_BiasAcc + J_Vel_wrt_BiasGyro*delta_BiasGyro;

    return predictVelocity_inertial(Pose1, Vel1,
        delta_vel_in_t0_corrected,
          dt12_, world_g_, world_rho_, world_omega_earth_);
  }

  static inline VELOCITY predictVelocity_inertial(const POSE& Pose1, const VELOCITY& Vel1,
      const Vector& delta_vel_in_t0,
      const double dt12, const Vector& world_g, const Vector& world_rho, const Vector& world_omega_earth) {

    const POSE& world_P1_body = Pose1;
      const VELOCITY& world_V1_body = Vel1;

      Vector body_deltaVel_body = delta_vel_in_t0;
      Vector world_deltaVel_body = world_P1_body.rotation().matrix() * body_deltaVel_body;

    VELOCITY VelDelta( world_deltaVel_body + world_g * dt12 );

    // Incorporate earth-related terms. Note - these are assumed to be constant between t1 and t2.
    VelDelta -= 2*skewSymmetric(world_rho + world_omega_earth)*world_V1_body * dt12;

    // Predict
    return Vel1 + VelDelta;

  }

  void predict(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1, POSE& Pose2, VELOCITY& Vel2) const {
    Pose2 = predictPose(Pose1, Vel1, Bias1);
    Vel2  = predictVelocity(Pose1, Vel1, Bias1);
  }

  POSE evaluatePoseError(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1, const POSE& Pose2, const VELOCITY& Vel2) const {
    // Predict
    POSE Pose2Pred = predictPose(Pose1, Vel1, Bias1);

    // Luca: difference between Pose2 and Pose2Pred
    POSE DiffPose( Pose2.rotation().between(Pose2Pred.rotation()),  Pose2Pred.translation() - Pose2.translation() );
//    DiffPose = Pose2.between(Pose2Pred);
    return DiffPose;
    // Calculate error
    //return Pose2.between(Pose2Pred);
  }

  VELOCITY evaluateVelocityError(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1, const POSE& Pose2, const VELOCITY& Vel2) const {
    // Predict
    VELOCITY Vel2Pred = predictVelocity(Pose1, Vel1, Bias1);

    // Calculate error
    return Vel2Pred-Vel2;
  }

  Vector evaluateError(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1, const POSE& Pose2, const VELOCITY& Vel2,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none,
      boost::optional<Matrix&> H4 = boost::none,
      boost::optional<Matrix&> H5 = boost::none) const override {

    // TODO: Write analytical derivative calculations
    // Jacobian w.r.t. Pose1
    if (H1){
      Matrix H1_Pose = numericalDerivative11<POSE, POSE>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluatePoseError,
                      this, std::placeholders::_1, Vel1, Bias1, Pose2, Vel2),
          Pose1);
      Matrix H1_Vel = numericalDerivative11<VELOCITY, POSE>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluateVelocityError,
                      this, std::placeholders::_1, Vel1, Bias1, Pose2, Vel2),
          Pose1);
      *H1 = stack(2, &H1_Pose, &H1_Vel);
    }

    // Jacobian w.r.t. Vel1
    if (H2){
      if (Vel1.size()!=3) throw std::runtime_error("Frank's hack to make this compile will not work if size != 3");
      Matrix H2_Pose = numericalDerivative11<POSE, Vector3>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluatePoseError,
                      this, Pose1, std::placeholders::_1, Bias1, Pose2, Vel2),
          Vel1);
      Matrix H2_Vel = numericalDerivative11<Vector3, Vector3>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluateVelocityError,
                      this, Pose1, std::placeholders::_1, Bias1, Pose2, Vel2),
          Vel1);
      *H2 = stack(2, &H2_Pose, &H2_Vel);
    }

    // Jacobian w.r.t. IMUBias1
    if (H3){
      Matrix H3_Pose = numericalDerivative11<POSE, IMUBIAS>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluatePoseError,
                      this, Pose1, Vel1, std::placeholders::_1, Pose2, Vel2),
          Bias1);
      Matrix H3_Vel = numericalDerivative11<VELOCITY, IMUBIAS>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluateVelocityError,
                      this, Pose1, Vel1, std::placeholders::_1, Pose2, Vel2),
          Bias1);
      *H3 = stack(2, &H3_Pose, &H3_Vel);
    }

    // Jacobian w.r.t. Pose2
    if (H4){
      Matrix H4_Pose = numericalDerivative11<POSE, POSE>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluatePoseError,
                      this, Pose1, Vel1, Bias1, std::placeholders::_1, Vel2),
          Pose2);
      Matrix H4_Vel = numericalDerivative11<VELOCITY, POSE>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluateVelocityError,
                      this, Pose1, Vel1, Bias1, std::placeholders::_1, Vel2),
          Pose2);
      *H4 = stack(2, &H4_Pose, &H4_Vel);
    }

    // Jacobian w.r.t. Vel2
    if (H5){
      if (Vel2.size()!=3) throw std::runtime_error("Frank's hack to make this compile will not work if size != 3");
      Matrix H5_Pose = numericalDerivative11<POSE, Vector3>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluatePoseError,
                      this, Pose1, Vel1, Bias1, Pose2, std::placeholders::_1),
          Vel2);
      Matrix H5_Vel = numericalDerivative11<Vector3, Vector3>(
          std::bind(&EquivInertialNavFactor_GlobalVel::evaluateVelocityError,
                      this, Pose1, Vel1, Bias1, Pose2, std::placeholders::_1),
          Vel2);
      *H5 = stack(2, &H5_Pose, &H5_Vel);
    }

    Vector ErrPoseVector(POSE::Logmap(evaluatePoseError(Pose1, Vel1, Bias1, Pose2, Vel2)));
    Vector ErrVelVector(evaluateVelocityError(Pose1, Vel1, Bias1, Pose2, Vel2));

    return concatVectors(2, &ErrPoseVector, &ErrVelVector);
  }



  static inline POSE PredictPoseFromPreIntegration(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1,
      const Vector& delta_pos_in_t0, const Vector3& delta_angles,
      double dt12, const Vector world_g, const Vector world_rho,
      const Vector& world_omega_earth, const Matrix& Jacobian_wrt_t0_Overall,
      const boost::optional<IMUBIAS>& Bias_initial = boost::none) {


    // Correct delta_pos_in_t0_ using (Bias1 - Bias_t0)
    Vector delta_BiasAcc  = Bias1.accelerometer();
    Vector delta_BiasGyro = Bias1.gyroscope();
    if (Bias_initial){
      delta_BiasAcc  -= Bias_initial->accelerometer();
      delta_BiasGyro -= Bias_initial->gyroscope();
    }

    Matrix J_Pos_wrt_BiasAcc  = Jacobian_wrt_t0_Overall.block(4,9,3,3);
    Matrix J_Pos_wrt_BiasGyro = Jacobian_wrt_t0_Overall.block(4,12,3,3);
    Matrix J_angles_wrt_BiasGyro = Jacobian_wrt_t0_Overall.block(0,12,3,3);

    /* Position term */
    Vector delta_pos_in_t0_corrected = delta_pos_in_t0 + J_Pos_wrt_BiasAcc*delta_BiasAcc + J_Pos_wrt_BiasGyro*delta_BiasGyro;

    /* Rotation term */
    Vector delta_angles_corrected = delta_angles + J_angles_wrt_BiasGyro*delta_BiasGyro;
    // Another alternative:
    //    Vector delta_angles_corrected = Rot3::Logmap( Rot3::Expmap(delta_angles_)*Rot3::Expmap(J_angles_wrt_BiasGyro*delta_BiasGyro) );

    return predictPose_inertial(Pose1, Vel1, delta_pos_in_t0_corrected, delta_angles_corrected, dt12, world_g, world_rho, world_omega_earth);
  }

  static inline VELOCITY PredictVelocityFromPreIntegration(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1,
      const Vector& delta_vel_in_t0, double dt12, const Vector world_g, const Vector world_rho,
      const Vector& world_omega_earth, const Matrix& Jacobian_wrt_t0_Overall,
      const boost::optional<IMUBIAS>& Bias_initial = boost::none) {

    // Correct delta_vel_in_t0_ using (Bias1 - Bias_t0)
    Vector delta_BiasAcc  = Bias1.accelerometer();
    Vector delta_BiasGyro = Bias1.gyroscope();
    if (Bias_initial){
      delta_BiasAcc  -= Bias_initial->accelerometer();
      delta_BiasGyro -= Bias_initial->gyroscope();
    }

    Matrix J_Vel_wrt_BiasAcc  = Jacobian_wrt_t0_Overall.block(6,9,3,3);
    Matrix J_Vel_wrt_BiasGyro = Jacobian_wrt_t0_Overall.block(6,12,3,3);

    Vector delta_vel_in_t0_corrected = delta_vel_in_t0 + J_Vel_wrt_BiasAcc*delta_BiasAcc + J_Vel_wrt_BiasGyro*delta_BiasGyro;

    return predictVelocity_inertial(Pose1, Vel1, delta_vel_in_t0_corrected, dt12, world_g, world_rho, world_omega_earth);
  }

  static inline void PredictFromPreIntegration(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1, POSE& Pose2, VELOCITY& Vel2,
      const Vector& delta_pos_in_t0, const Vector& delta_vel_in_t0, const Vector3& delta_angles,
      double dt12, const Vector world_g, const Vector world_rho,
      const Vector& world_omega_earth, const Matrix& Jacobian_wrt_t0_Overall,
      const boost::optional<IMUBIAS>& Bias_initial = boost::none) {

    Pose2 = PredictPoseFromPreIntegration(Pose1, Vel1, Bias1, delta_pos_in_t0, delta_angles, dt12, world_g, world_rho, world_omega_earth, Jacobian_wrt_t0_Overall, Bias_initial);
    Vel2  = PredictVelocityFromPreIntegration(Pose1, Vel1, Bias1, delta_vel_in_t0, dt12, world_g, world_rho, world_omega_earth, Jacobian_wrt_t0_Overall, Bias_initial);
  }


  static inline void PreIntegrateIMUObservations(const Vector& msr_acc_t, const Vector& msr_gyro_t, const double msr_dt,
      Vector& delta_pos_in_t0, Vector3& delta_angles, Vector& delta_vel_in_t0, double& delta_t,
      const noiseModel::Gaussian::shared_ptr& model_continuous_overall,
      Matrix& EquivCov_Overall, Matrix& Jacobian_wrt_t0_Overall, const IMUBIAS Bias_t0 = IMUBIAS(),
      boost::optional<POSE> p_body_P_sensor = boost::none){
    // Note: all delta terms refer to an IMU\sensor system at t0
    // Note: Earth-related terms are not accounted here but are incorporated in predict functions.

    POSE body_P_sensor = POSE();
    bool flag_use_body_P_sensor = false;
    if (p_body_P_sensor){
      body_P_sensor = *p_body_P_sensor;
      flag_use_body_P_sensor = true;
    }

    delta_pos_in_t0 = PreIntegrateIMUObservations_delta_pos(msr_dt, delta_pos_in_t0, delta_vel_in_t0);
    delta_vel_in_t0 = PreIntegrateIMUObservations_delta_vel(msr_gyro_t, msr_acc_t, msr_dt, delta_angles, delta_vel_in_t0, flag_use_body_P_sensor, body_P_sensor, Bias_t0);
    delta_angles = PreIntegrateIMUObservations_delta_angles(msr_gyro_t, msr_dt, delta_angles, flag_use_body_P_sensor, body_P_sensor, Bias_t0);

    delta_t += msr_dt;

    // Update EquivCov_Overall
    Matrix Z_3x3 = Z_3x3;
    Matrix I_3x3 = I_3x3;

    Matrix H_pos_pos = numericalDerivative11<Vector3, Vector3>(
        std::bind(&PreIntegrateIMUObservations_delta_pos, msr_dt,
                    std::placeholders::_1, delta_vel_in_t0),
        delta_pos_in_t0);
    Matrix H_pos_vel = numericalDerivative11<Vector3, Vector3>(
        std::bind(&PreIntegrateIMUObservations_delta_pos, msr_dt,
                    delta_pos_in_t0, std::placeholders::_1),
        delta_vel_in_t0);
    Matrix H_pos_angles = Z_3x3;
    Matrix H_pos_bias = collect(2, &Z_3x3, &Z_3x3);

    Matrix H_vel_vel = numericalDerivative11<Vector3, Vector3>(
        std::bind(&PreIntegrateIMUObservations_delta_vel, msr_gyro_t,
                    msr_acc_t, msr_dt, delta_angles, std::placeholders::_1,
                    flag_use_body_P_sensor, body_P_sensor, Bias_t0),
        delta_vel_in_t0);
    Matrix H_vel_angles = numericalDerivative11<Vector3, Vector3>(
        std::bind(&PreIntegrateIMUObservations_delta_vel, msr_gyro_t,
                    msr_acc_t, msr_dt, std::placeholders::_1, delta_vel_in_t0,
                    flag_use_body_P_sensor, body_P_sensor, Bias_t0),
        delta_angles);
    Matrix H_vel_bias = numericalDerivative11<Vector3, IMUBIAS>(
        std::bind(&PreIntegrateIMUObservations_delta_vel, msr_gyro_t,
                    msr_acc_t, msr_dt, delta_angles, delta_vel_in_t0,
                    flag_use_body_P_sensor, body_P_sensor,
                    std::placeholders::_1),
        Bias_t0);
    Matrix H_vel_pos = Z_3x3;

    Matrix H_angles_angles = numericalDerivative11<Vector3, Vector3>(
        std::bind(&PreIntegrateIMUObservations_delta_angles, msr_gyro_t,
                    msr_dt, std::placeholders::_1, flag_use_body_P_sensor,
                    body_P_sensor, Bias_t0),
        delta_angles);
    Matrix H_angles_bias = numericalDerivative11<Vector3, IMUBIAS>(
        std::bind(&PreIntegrateIMUObservations_delta_angles, msr_gyro_t,
                    msr_dt, delta_angles, flag_use_body_P_sensor, body_P_sensor,
                    std::placeholders::_1),
        Bias_t0);
    Matrix H_angles_pos = Z_3x3;
    Matrix H_angles_vel = Z_3x3;

    Matrix F_angles = collect(4, &H_angles_angles, &H_angles_pos, &H_angles_vel, &H_angles_bias);
    Matrix F_pos    = collect(4, &H_pos_angles, &H_pos_pos, &H_pos_vel, &H_pos_bias);
    Matrix F_vel    = collect(4, &H_vel_angles, &H_vel_pos, &H_vel_vel, &H_vel_bias);
    Matrix F_bias_a = collect(5, &Z_3x3, &Z_3x3, &Z_3x3, &I_3x3, &Z_3x3);
    Matrix F_bias_g = collect(5, &Z_3x3, &Z_3x3, &Z_3x3, &Z_3x3, &I_3x3);
    Matrix F = stack(5, &F_angles, &F_pos, &F_vel, &F_bias_a, &F_bias_g);


    noiseModel::Gaussian::shared_ptr model_discrete_curr = calc_descrete_noise_model(model_continuous_overall, msr_dt );
    Matrix Q_d = (model_discrete_curr->R().transpose() * model_discrete_curr->R()).inverse();

    EquivCov_Overall = F * EquivCov_Overall * F.transpose() + Q_d;
    // Luca: force identity covariance matrix (for testing purposes)
    // EquivCov_Overall = Matrix::Identity(15,15);

    // Update Jacobian_wrt_t0_Overall
    Jacobian_wrt_t0_Overall = F * Jacobian_wrt_t0_Overall;
  }

  static inline Vector PreIntegrateIMUObservations_delta_pos(const double msr_dt,
      const Vector& delta_pos_in_t0, const Vector& delta_vel_in_t0){

    // Note: all delta terms refer to an IMU\sensor system at t0
    // Note: delta_vel_in_t0 is already in body frame, so no need to use the body_P_sensor transformation here.

    return delta_pos_in_t0 + delta_vel_in_t0 * msr_dt;
  }



  static inline Vector PreIntegrateIMUObservations_delta_vel(const Vector& msr_gyro_t, const Vector& msr_acc_t, const double msr_dt,
      const Vector3& delta_angles, const Vector& delta_vel_in_t0, const bool flag_use_body_P_sensor, const POSE& body_P_sensor,
      IMUBIAS Bias_t0 = IMUBIAS()){

    // Note: all delta terms refer to an IMU\sensor system at t0

    // Calculate the corrected measurements using the Bias object
    Vector AccCorrected  = Bias_t0.correctAccelerometer(msr_acc_t);
    Vector body_t_a_body;
    if (flag_use_body_P_sensor){
      Matrix body_R_sensor = body_P_sensor.rotation().matrix();

      Vector GyroCorrected(Bias_t0.correctGyroscope(msr_gyro_t));

      Vector body_omega_body = body_R_sensor * GyroCorrected;
      Matrix body_omega_body__cross = skewSymmetric(body_omega_body);

      body_t_a_body = body_R_sensor * AccCorrected - body_omega_body__cross * body_omega_body__cross * body_P_sensor.translation().vector();
    } else{
      body_t_a_body = AccCorrected;
    }

    Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);

    return delta_vel_in_t0 + R_t_to_t0.matrix() * body_t_a_body * msr_dt;
  }


  static inline Vector PreIntegrateIMUObservations_delta_angles(const Vector& msr_gyro_t, const double msr_dt,
      const Vector3& delta_angles, const bool flag_use_body_P_sensor, const POSE& body_P_sensor,
      IMUBIAS Bias_t0 = IMUBIAS()){

    // Note: all delta terms refer to an IMU\sensor system at t0

    // Calculate the corrected measurements using the Bias object
    Vector GyroCorrected = Bias_t0.correctGyroscope(msr_gyro_t);

    Vector body_t_omega_body;
    if (flag_use_body_P_sensor){
      body_t_omega_body = body_P_sensor.rotation().matrix() * GyroCorrected;
    } else {
      body_t_omega_body = GyroCorrected;
    }

    Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);

    R_t_to_t0    = R_t_to_t0 * Rot3::Expmap( body_t_omega_body*msr_dt );
    return Rot3::Logmap(R_t_to_t0);
  }


  static inline noiseModel::Gaussian::shared_ptr CalcEquivalentNoiseCov(const noiseModel::Gaussian::shared_ptr& gaussian_acc, const noiseModel::Gaussian::shared_ptr& gaussian_gyro,
      const noiseModel::Gaussian::shared_ptr& gaussian_process){

    Matrix cov_acc = ( gaussian_acc->R().transpose() * gaussian_acc->R() ).inverse();
    Matrix cov_gyro = ( gaussian_gyro->R().transpose() * gaussian_gyro->R() ).inverse();
    Matrix cov_process = ( gaussian_process->R().transpose() * gaussian_process->R() ).inverse();

    cov_process.block(0,0, 3,3) += cov_gyro;
    cov_process.block(6,6, 3,3) += cov_acc;

    return noiseModel::Gaussian::Covariance(cov_process);
  }

  static inline void CalcEquivalentNoiseCov_DifferentParts(const noiseModel::Gaussian::shared_ptr& gaussian_acc, const noiseModel::Gaussian::shared_ptr& gaussian_gyro,
      const noiseModel::Gaussian::shared_ptr& gaussian_process,
      Matrix& cov_acc, Matrix& cov_gyro, Matrix& cov_process_without_acc_gyro){

    cov_acc = ( gaussian_acc->R().transpose() * gaussian_acc->R() ).inverse();
    cov_gyro = ( gaussian_gyro->R().transpose() * gaussian_gyro->R() ).inverse();
    cov_process_without_acc_gyro = ( gaussian_process->R().transpose() * gaussian_process->R() ).inverse();
  }

  static inline void Calc_g_rho_omega_earth_NED(const Vector& Pos_NED, const Vector& Vel_NED, const Vector& LatLonHeight_IC, const Vector& Pos_NED_Initial,
      Vector& g_NED, Vector& rho_NED, Vector& omega_earth_NED) {

    Matrix ENU_to_NED = (Matrix(3, 3) <<
        0.0,  1.0,  0.0,
        1.0,  0.0,  0.0,
        0.0,  0.0, -1.0).finished();

    Matrix NED_to_ENU = (Matrix(3, 3) <<
        0.0,  1.0,  0.0,
        1.0,  0.0,  0.0,
        0.0,  0.0, -1.0).finished();

    // Convert incoming parameters to ENU
    Vector Pos_ENU = NED_to_ENU * Pos_NED;
    Vector Vel_ENU = NED_to_ENU * Vel_NED;
    Vector Pos_ENU_Initial = NED_to_ENU * Pos_NED_Initial;

    // Call ENU version
    Vector g_ENU;
    Vector rho_ENU;
    Vector omega_earth_ENU;
    Calc_g_rho_omega_earth_ENU(Pos_ENU, Vel_ENU, LatLonHeight_IC, Pos_ENU_Initial, g_ENU, rho_ENU, omega_earth_ENU);

    // Convert output to NED
    g_NED = ENU_to_NED * g_ENU;
    rho_NED = ENU_to_NED * rho_ENU;
    omega_earth_NED = ENU_to_NED * omega_earth_ENU;
  }

  static inline void Calc_g_rho_omega_earth_ENU(const Vector& Pos_ENU, const Vector& Vel_ENU, const Vector& LatLonHeight_IC, const Vector& Pos_ENU_Initial,
      Vector& g_ENU, Vector& rho_ENU, Vector& omega_earth_ENU){
    double R0 = 6.378388e6;
    double e = 1/297;
    double Re( R0*( 1-e*(sin( LatLonHeight_IC(0) ))*(sin( LatLonHeight_IC(0) )) ) );

    // Calculate current lat, lon
    Vector delta_Pos_ENU(Pos_ENU - Pos_ENU_Initial);
    double delta_lat(delta_Pos_ENU(1)/Re);
    double delta_lon(delta_Pos_ENU(0)/(Re*cos(LatLonHeight_IC(0))));
    double lat_new(LatLonHeight_IC(0) + delta_lat);
    double lon_new(LatLonHeight_IC(1) + delta_lon);

    // Rotation of lon about z axis
    Rot3 C1(cos(lon_new), sin(lon_new), 0.0,
        -sin(lon_new), cos(lon_new), 0.0,
        0.0, 0.0, 1.0);

    // Rotation of lat about y axis
    Rot3 C2(cos(lat_new), 0.0, sin(lat_new),
        0.0, 1.0, 0.0,
        -sin(lat_new), 0.0, cos(lat_new));

    Rot3 UEN_to_ENU(0, 1, 0,
        0, 0, 1,
        1, 0, 0);

    Rot3 R_ECEF_to_ENU( UEN_to_ENU * C2 * C1 );

    Vector omega_earth_ECEF(Vector3(0.0, 0.0, 7.292115e-5));
    omega_earth_ENU = R_ECEF_to_ENU.matrix() * omega_earth_ECEF;

    // Calculating g
    double height(LatLonHeight_IC(2));
    double EQUA_RADIUS = 6378137.0;        // equatorial radius of the earth; WGS-84
    double ECCENTRICITY = 0.0818191908426;  // eccentricity of the earth ellipsoid
    double e2( pow(ECCENTRICITY,2) );
    double den( 1-e2*pow(sin(lat_new),2) );
    double Rm( (EQUA_RADIUS*(1-e2))/( pow(den,(3/2)) ) );
    double Rp( EQUA_RADIUS/( sqrt(den) ) );
    double Ro( sqrt(Rp*Rm) );           // mean earth radius of curvature
    double g0( 9.780318*( 1 + 5.3024e-3 * pow(sin(lat_new),2) - 5.9e-6 * pow(sin(2*lat_new),2) ) );
    double g_calc( g0/( pow(1 + height/Ro, 2) ) );
    g_ENU = (Vector(3) << 0.0, 0.0, -g_calc).finished();


    // Calculate rho
    double Ve( Vel_ENU(0) );
    double Vn( Vel_ENU(1) );
    double rho_E = -Vn/(Rm + height);
    double rho_N = Ve/(Rp + height);
    double rho_U = Ve*tan(lat_new)/(Rp + height);
    rho_ENU = (Vector(3) << rho_E, rho_N, rho_U).finished();
  }

  static inline noiseModel::Gaussian::shared_ptr calc_descrete_noise_model(const noiseModel::Gaussian::shared_ptr& model, double delta_t){
      /* Q_d (approx)= Q * delta_t */
      /* In practice, square root of the information matrix is represented, so that:
       *  R_d (approx)= R / sqrt(delta_t)
       * */
      return noiseModel::Gaussian::SqrtInformation(model->R()/sqrt(delta_t));
    }
private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NonlinearFactor2",
        boost::serialization::base_object<Base>(*this));
  }



}; // \class EquivInertialNavFactor_GlobalVel

} /// namespace gtsam

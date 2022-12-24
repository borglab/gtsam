/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   InertialNavFactor_GlobalVelocity.h
 *  @author Vadim Indelman, Stephen Williams
 *  @brief  Inertial navigation factor (velocity in the global frame)
 *  @date   Sept 13, 2012
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
class InertialNavFactor_GlobalVelocity : public NoiseModelFactorN<POSE, VELOCITY, IMUBIAS, POSE, VELOCITY> {

private:

  typedef InertialNavFactor_GlobalVelocity<POSE, VELOCITY, IMUBIAS> This;
  typedef NoiseModelFactorN<POSE, VELOCITY, IMUBIAS, POSE, VELOCITY> Base;

  Vector measurement_acc_;
  Vector measurement_gyro_;
  double dt_;

  Vector world_g_;
  Vector world_rho_;
  Vector world_omega_earth_;

  boost::optional<POSE> body_P_sensor_; // The pose of the sensor in the body frame

public:

  // shorthand for a smart pointer to a factor
  typedef typename boost::shared_ptr<InertialNavFactor_GlobalVelocity> shared_ptr;

  /** default constructor - only use for serialization */
  InertialNavFactor_GlobalVelocity() {}

  /** Constructor */
  InertialNavFactor_GlobalVelocity(const Key& Pose1, const Key& Vel1, const Key& IMUBias1, const Key& Pose2, const Key& Vel2,
      const Vector& measurement_acc, const Vector& measurement_gyro, const double measurement_dt, const Vector world_g, const Vector world_rho,
      const Vector& world_omega_earth, const noiseModel::Gaussian::shared_ptr& model_continuous, boost::optional<POSE> body_P_sensor = boost::none) :
        Base(calc_descrete_noise_model(model_continuous, measurement_dt ),
            Pose1, Vel1, IMUBias1, Pose2, Vel2), measurement_acc_(measurement_acc), measurement_gyro_(measurement_gyro),
            dt_(measurement_dt), world_g_(world_g), world_rho_(world_rho), world_omega_earth_(world_omega_earth), body_P_sensor_(body_P_sensor) {  }

  ~InertialNavFactor_GlobalVelocity() override {}

  /** implement functions needed for Testable */

  /** print */
  void print(const std::string& s = "InertialNavFactor_GlobalVelocity", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "("
        << keyFormatter(this->template key<1>()) << ","
        << keyFormatter(this->template key<2>()) << ","
        << keyFormatter(this->template key<3>()) << ","
        << keyFormatter(this->template key<4>()) << ","
        << keyFormatter(this->template key<5>()) << "\n";
    std::cout << "acc measurement: " << this->measurement_acc_.transpose() << std::endl;
    std::cout << "gyro measurement: " << this->measurement_gyro_.transpose() << std::endl;
    std::cout << "dt: " << this->dt_ << std::endl;
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
      && (measurement_acc_ - e->measurement_acc_).norm() < tol
      && (measurement_gyro_ - e->measurement_gyro_).norm() < tol
      && (dt_ - e->dt_) < tol
      && (world_g_ - e->world_g_).norm() < tol
      && (world_rho_ - e->world_rho_).norm() < tol
      && (world_omega_earth_ - e->world_omega_earth_).norm() < tol
      && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  POSE predictPose(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1) const {
    // Calculate the corrected measurements using the Bias object
    Vector GyroCorrected(Bias1.correctGyroscope(measurement_gyro_));

    const POSE& world_P1_body = Pose1;
    const VELOCITY& world_V1_body = Vel1;

    // Calculate the acceleration and angular velocity of the body in the body frame (including earth-related rotations)
    Vector body_omega_body;
    if(body_P_sensor_) {
      body_omega_body = body_P_sensor_->rotation().matrix() * GyroCorrected;
    } else {
      body_omega_body = GyroCorrected;
    }

    // Convert earth-related terms into the body frame
    Matrix body_R_world(world_P1_body.rotation().inverse().matrix());
    Vector body_rho = body_R_world * world_rho_;
    Vector body_omega_earth = body_R_world * world_omega_earth_;

    // Correct for earth-related terms
    body_omega_body -= body_rho + body_omega_earth;

    // The velocity is in the global frame, so composing Pose1 with v*dt is incorrect
    return POSE(Pose1.rotation() * POSE::Rotation::Expmap(body_omega_body*dt_), Pose1.translation() + typename POSE::Translation(world_V1_body*dt_));
  }

  VELOCITY predictVelocity(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1) const {
    // Calculate the corrected measurements using the Bias object
    Vector AccCorrected(Bias1.correctAccelerometer(measurement_acc_));

    const POSE& world_P1_body = Pose1;
    const VELOCITY& world_V1_body = Vel1;

    // Calculate the acceleration and angular velocity of the body in the body frame (including earth-related rotations)
    Vector body_a_body, body_omega_body;
    if(body_P_sensor_) {
      Matrix body_R_sensor = body_P_sensor_->rotation().matrix();

      Vector GyroCorrected(Bias1.correctGyroscope(measurement_gyro_));
      body_omega_body = body_R_sensor * GyroCorrected;
      Matrix body_omega_body__cross = skewSymmetric(body_omega_body);
      body_a_body = body_R_sensor * AccCorrected - body_omega_body__cross * body_omega_body__cross * body_P_sensor_->translation();
    } else {
      body_a_body = AccCorrected;
    }

    // Correct for earth-related terms
    Vector world_a_body = world_P1_body.rotation().matrix() * body_a_body + world_g_ - 2*skewSymmetric(world_rho_ + world_omega_earth_)*world_V1_body;

    // Calculate delta in the body frame
    VELOCITY VelDelta(world_a_body*dt_);

    // Predict
    return Vel1 + VelDelta;
  }

  void predict(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1, POSE& Pose2, VELOCITY& Vel2) const {
    Pose2 = predictPose(Pose1, Vel1, Bias1);
    Vel2 = predictVelocity(Pose1, Vel1, Bias1);
  }

  POSE evaluatePoseError(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1, const POSE& Pose2, const VELOCITY& Vel2) const {
    // Predict
    POSE Pose2Pred = predictPose(Pose1, Vel1, Bias1);

    // Calculate error
    return Pose2.between(Pose2Pred);
  }

  VELOCITY evaluateVelocityError(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1, const POSE& Pose2, const VELOCITY& Vel2) const {
    // Predict
    VELOCITY Vel2Pred = predictVelocity(Pose1, Vel1, Bias1);

    // Calculate error
    return Vel2Pred - Vel2;
  }

  /** implement functions needed to derive from Factor */
  Vector evaluateError(const POSE& Pose1, const VELOCITY& Vel1, const IMUBIAS& Bias1, const POSE& Pose2, const VELOCITY& Vel2,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none,
      boost::optional<Matrix&> H4 = boost::none,
      boost::optional<Matrix&> H5 = boost::none) const override {

    // TODO: Write analytical derivative calculations
    // Jacobian w.r.t. Pose1
    if (H1){
      Matrix H1_Pose = gtsam::numericalDerivative11<POSE, POSE>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluatePoseError,
                      this, std::placeholders::_1, Vel1, Bias1, Pose2, Vel2),
          Pose1);
      Matrix H1_Vel = gtsam::numericalDerivative11<VELOCITY, POSE>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluateVelocityError,
                      this, std::placeholders::_1, Vel1, Bias1, Pose2, Vel2),
          Pose1);
      *H1 = stack(2, &H1_Pose, &H1_Vel);
    }

    // Jacobian w.r.t. Vel1
    if (H2){
      if (Vel1.size()!=3) throw std::runtime_error("Frank's hack to make this compile will not work if size != 3");
      Matrix H2_Pose = gtsam::numericalDerivative11<POSE, Vector3>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluatePoseError,
                      this, Pose1, std::placeholders::_1, Bias1, Pose2, Vel2),
          Vel1);
      Matrix H2_Vel = gtsam::numericalDerivative11<Vector3, Vector3>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluateVelocityError,
                      this, Pose1, std::placeholders::_1, Bias1, Pose2, Vel2),
          Vel1);
      *H2 = stack(2, &H2_Pose, &H2_Vel);
    }

    // Jacobian w.r.t. IMUBias1
    if (H3){
      Matrix H3_Pose = gtsam::numericalDerivative11<POSE, IMUBIAS>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluatePoseError,
                      this, Pose1, Vel1, std::placeholders::_1, Pose2, Vel2),
          Bias1);
      Matrix H3_Vel = gtsam::numericalDerivative11<VELOCITY, IMUBIAS>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluateVelocityError,
                      this, Pose1, Vel1, std::placeholders::_1, Pose2, Vel2),
          Bias1);
      *H3 = stack(2, &H3_Pose, &H3_Vel);
    }

    // Jacobian w.r.t. Pose2
    if (H4){
      Matrix H4_Pose = gtsam::numericalDerivative11<POSE, POSE>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluatePoseError,
                      this, Pose1, Vel1, Bias1, std::placeholders::_1, Vel2),
          Pose2);
      Matrix H4_Vel = gtsam::numericalDerivative11<VELOCITY, POSE>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluateVelocityError,
                      this, Pose1, Vel1, Bias1, std::placeholders::_1, Vel2),
          Pose2);
      *H4 = stack(2, &H4_Pose, &H4_Vel);
    }

    // Jacobian w.r.t. Vel2
    if (H5){
      if (Vel2.size()!=3) throw std::runtime_error("Frank's hack to make this compile will not work if size != 3");
      Matrix H5_Pose = gtsam::numericalDerivative11<POSE, Vector3>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluatePoseError,
                      this, Pose1, Vel1, Bias1, Pose2, std::placeholders::_1),
          Vel2);
      Matrix H5_Vel = gtsam::numericalDerivative11<Vector3, Vector3>(
          std::bind(&InertialNavFactor_GlobalVelocity::evaluateVelocityError,
                      this, Pose1, Vel1, Bias1, Pose2, std::placeholders::_1),
          Vel2);
      *H5 = stack(2, &H5_Pose, &H5_Vel);
    }

    Vector ErrPoseVector(POSE::Logmap(evaluatePoseError(Pose1, Vel1, Bias1, Pose2, Vel2)));
    Vector ErrVelVector(evaluateVelocityError(Pose1, Vel1, Bias1, Pose2, Vel2));

    return concatVectors(2, &ErrPoseVector, &ErrVelVector);
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
      return noiseModel::Gaussian::SqrtInformation(model->R()/std::sqrt(delta_t));
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NonlinearFactor2",
        boost::serialization::base_object<Base>(*this));
  }

}; // \class InertialNavFactor_GlobalVelocity

/// traits
template<class POSE, class VELOCITY, class IMUBIAS>
struct traits<InertialNavFactor_GlobalVelocity<POSE, VELOCITY, IMUBIAS> > :
    public Testable<InertialNavFactor_GlobalVelocity<POSE, VELOCITY, IMUBIAS> > {
};

} /// namespace aspn

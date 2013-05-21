/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ImuFactor.h
 *  @author Luca Carlone, Stephen Williams
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/debug.h>

/* External or standard includes */
#include <ostream>


namespace gtsam {

  /**
   * 
   * @addtogroup SLAM
   */
  class ImuFactor: public NoiseModelFactor5<Pose3,LieVector,Pose3,LieVector,imuBias::ConstantBias> {

  public:

    /** Struct to store results of preintegrating IMU measurements.  Can be build
     * incrementally so as to avoid costly integration at time of factor
     * construction. */

    /** Right Jacobian for SO(3) */
    static Matrix3 rightJacobianExpMapSO3(const Vector3& x)    {
      // x is the axis angle representation (exponential coordinates) for a rotation
      const Matrix3 X = skewSymmetric(x); // element of Lie algebra so(3): X = x^
      double normx = norm_2(x); // rotation angle
      const Matrix3 Jr = Matrix3::Identity() - ((1-cos(normx))/(normx*normx)) * X +
          ((normx-sin(normx))/(normx*normx*normx)) * X * X; // right Jacobian
      return Jr;
    }

    /** Inverse of the Right Jacobian for SO(3) */
    static Matrix3 rightJacobianExpMapSO3inverse(const Vector3& x)    {
      // x is the axis angle representation (exponential coordinates) for a rotation
      const Matrix3 X = skewSymmetric(x); // element of Lie algebra so(3): X = x^
      double normx = norm_2(x); // rotation angle
      const Matrix3 Jrinv = Matrix3::Identity() +
          0.5 * X + (1/(normx*normx) - (1+cos(normx))/(2*normx * sin(normx))   ) * X * X;
      return Jrinv;
    }


    class PreintegratedMeasurements {
    public:
      imuBias::ConstantBias biasHat; ///< Acceleration and angular rate bias values used during preintegration
      Matrix measurementCovariance; ///< Covariance of the vector [measuredAcc measuredOmega integrationError] in R^(9X9)

      Vector3 deltaPij; ///< Preintegrated relative position (in frame i)
      Vector3 deltaVij; ///< Preintegrated relative velocity (in global frame)
      Rot3 deltaRij; ///< Preintegrated relative orientation (in frame i)
      double deltaTij; ///< Time interval from i to j

      Matrix3 delPdelBiasAcc; ///< Jacobian of preintegrated position w.r.t. acceleration bias
      Matrix3 delPdelBiasOmega; ///< Jacobian of preintegrated position w.r.t. angular rate bias
      Matrix3 delVdelBiasAcc; ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
      Matrix3 delVdelBiasOmega; ///< Jacobian of preintegrated velocity w.r.t. angular rate bias
      Matrix3 delRdelBiasOmega; ///< Jacobian of preintegrated rotation w.r.t. angular rate bias

      /** Default constructor, initialize with no IMU measurements */
      PreintegratedMeasurements(
          const imuBias::ConstantBias& bias, ///< Current estimate of acceleration and rotation rate biases
          const Matrix3& measuredAccCovariance, ///< Covariance matrix of measuredAcc
          const Matrix3& measuredOmegaCovariance, ///< Covariance matrix of measuredAcc
          const Matrix3& integrationErrorCovariance ///< Covariance matrix of measuredAcc
      ) : biasHat(bias), measurementCovariance(9,9), deltaPij(Vector3::Zero()), deltaVij(Vector3::Zero()), deltaTij(0.0),
      delPdelBiasAcc(Matrix3::Zero()), delPdelBiasOmega(Matrix3::Zero()),
      delVdelBiasAcc(Matrix3::Zero()), delVdelBiasOmega(Matrix3::Zero()),
      delRdelBiasOmega(Matrix3::Zero())
      {
        measurementCovariance << measuredAccCovariance , Matrix3::Zero(), Matrix3::Zero(),
            Matrix3::Zero(), measuredOmegaCovariance,  Matrix3::Zero(),
            Matrix3::Zero(),   Matrix3::Zero(),   integrationErrorCovariance;
      }

      /** print */
      void print(const std::string& s = "Preintegrated Measurements:") const {
        std::cout << s << std::endl;
        biasHat.print("  biasHat");
        std::cout << "  deltaTij " << deltaTij << std::endl;
        std::cout << "  deltaPij [ " << deltaPij.transpose() << " ]" << std::endl;
        std::cout << "  deltaVij [ " << deltaVij.transpose() << " ]" << std::endl;
        deltaRij.print("  deltaRij ");
      }

      /** equals */
      bool equals(const PreintegratedMeasurements& expected, double tol=1e-9) const {
        return biasHat.equals(expected.biasHat, tol)
            && equal_with_abs_tol(measurementCovariance, expected.measurementCovariance, tol)
            && equal_with_abs_tol(deltaPij, expected.deltaPij, tol)
            && equal_with_abs_tol(deltaVij, expected.deltaVij, tol)
            && deltaRij.equals(expected.deltaRij, tol)
            && std::fabs(deltaTij - expected.deltaTij) < tol
            && equal_with_abs_tol(delPdelBiasAcc, expected.delPdelBiasAcc, tol)
            && equal_with_abs_tol(delPdelBiasOmega, expected.delPdelBiasOmega, tol)
            && equal_with_abs_tol(delVdelBiasAcc, expected.delVdelBiasAcc, tol)
            && equal_with_abs_tol(delVdelBiasOmega, expected.delVdelBiasOmega, tol)
            && equal_with_abs_tol(delRdelBiasOmega, expected.delRdelBiasOmega, tol);
      }

      /** Add a single IMU measurement to the preintegration. */
      void integrateMeasurement(
          const Vector3& measuredAcc, ///< Measured linear acceleration (in body frame)
          const Vector3& measuredOmega, ///< Measured angular velocity (in body frame)
          double deltaT ///< Time step
      ) {
        // NOTE: order is important here because each update uses old values.

        const Rot3 Rincr = Rot3::Expmap((biasHat.correctGyroscope(measuredOmega)) * deltaT);

        // Update Jacobians
        delPdelBiasAcc += delVdelBiasAcc * deltaT - 0.5 * deltaRij.matrix() * deltaT*deltaT;
        delPdelBiasOmega += delVdelBiasOmega * deltaT - 0.5 * deltaRij.matrix()
          * skewSymmetric(biasHat.correctAccelerometer(measuredAcc)) * deltaT*deltaT * delRdelBiasOmega;
        delVdelBiasAcc += -deltaRij.matrix() * deltaT;
        delVdelBiasOmega += -deltaRij.matrix() * skewSymmetric(biasHat.correctAccelerometer(measuredAcc)) * deltaT * delRdelBiasOmega;

        const Vector3 x = biasHat.correctGyroscope(measuredOmega) * deltaT; // parametrization of so(3)
        const Matrix3 Jr = rightJacobianExpMapSO3(x);
        delRdelBiasOmega = Rincr.inverse().matrix() * delRdelBiasOmega - Jr  * deltaT;

        // Update preintegrated measurements
        deltaPij += deltaVij * deltaT + 0.5 * deltaRij.matrix() * biasHat.correctAccelerometer(measuredAcc) * deltaT*deltaT;
        deltaVij += deltaRij.matrix() * biasHat.correctAccelerometer(measuredAcc) * deltaT;
        deltaRij = deltaRij * Rincr;
        deltaTij += deltaT;
      }

      /** Calculate the covariance of the preintegrated measurements */
      Matrix preintegratedMeasurementsCovariance() const {
        Matrix Jpreintegrated(9,9);
        Jpreintegrated <<
            //deltaP VS (measuredAcc   measuredOmega   intError)
            - delPdelBiasAcc,  - delPdelBiasOmega, - delPdelBiasAcc,
            //deltaV VS (measuredAcc   measuredOmega   intError)
            - delVdelBiasAcc,  - delVdelBiasOmega, Matrix3::Zero(),
            //deltaR VS (measuredAcc   measuredOmega   intError)
            Matrix3::Zero(), -delRdelBiasOmega, Matrix3::Zero();

        return Jpreintegrated * measurementCovariance * Jpreintegrated.transpose();
      }

    private:
      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(biasHat);
        ar & BOOST_SERIALIZATION_NVP(measurementCovariance);
        ar & BOOST_SERIALIZATION_NVP(deltaPij);
        ar & BOOST_SERIALIZATION_NVP(deltaVij);
        ar & BOOST_SERIALIZATION_NVP(deltaRij);
        ar & BOOST_SERIALIZATION_NVP(deltaTij);
        ar & BOOST_SERIALIZATION_NVP(delPdelBiasAcc);
        ar & BOOST_SERIALIZATION_NVP(delPdelBiasOmega);
        ar & BOOST_SERIALIZATION_NVP(delVdelBiasAcc);
        ar & BOOST_SERIALIZATION_NVP(delVdelBiasOmega);
        ar & BOOST_SERIALIZATION_NVP(delRdelBiasOmega);
      }
    };

  private:

    typedef ImuFactor This;
    typedef NoiseModelFactor5<Pose3,LieVector,Pose3,LieVector,imuBias::ConstantBias> Base;

    PreintegratedMeasurements preintegratedMeasurements_;
    Vector3 gravity_;
    Vector3 omegaCoriolis_;
    boost::optional<Pose3> body_P_sensor_;        ///< The pose of the sensor in the body frame

  public:

    /** Shorthand for a smart pointer to a factor */
    typedef boost::shared_ptr<ImuFactor> shared_ptr;

    /** Default constructor - only use for serialization */
    ImuFactor() : preintegratedMeasurements_(imuBias::ConstantBias(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero()) {}

    /** Constructor */
    ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
        const PreintegratedMeasurements& preintegratedMeasurements, const Vector3& gravity, const Vector3& omegaCoriolis,
        const SharedNoiseModel& model, boost::optional<Pose3> body_P_sensor = boost::none) :
      Base(model, pose_i, vel_i, pose_j, vel_j, bias),
      preintegratedMeasurements_(preintegratedMeasurements),
      gravity_(gravity),
      omegaCoriolis_(omegaCoriolis),
      body_P_sensor_(body_P_sensor) {
    }

    virtual ~ImuFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "ImuFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ","
          << keyFormatter(this->key3()) << ","
          << keyFormatter(this->key4()) << ","
          << keyFormatter(this->key5()) << ")\n";
      preintegratedMeasurements_.print("  preintegrated measurements:");
      std::cout << "  gravity: [ " << gravity_.transpose() << " ]" << std::endl;
      std::cout << "  omegaCoriolis: [ " << omegaCoriolis_.transpose() << " ]" << std::endl;
      this->noiseModel_->print("  noise model: ");
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol)
          && preintegratedMeasurements_.equals(e->preintegratedMeasurements_)
          && equal_with_abs_tol(gravity_, e->gravity_, tol)
          && equal_with_abs_tol(omegaCoriolis_, e->omegaCoriolis_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /** Access the preintegrated measurements. */
    const PreintegratedMeasurements& preintegratedMeasurements() const {
      return preintegratedMeasurements_; }

    /** Predict the pose and velocity at {j} using the pre-integrated measurement */
    // TODO: This currently repeats the equations in 'evaluateError'. Refactor to remove the duplication
    void predict(const Pose3& pose_i, const LieVector& vel_i, Pose3& pose_j, LieVector& vel_j, const imuBias::ConstantBias& bias) const {

      // Calculate bias delta between preintegrated value and user-provided value
      const double& deltaTij = preintegratedMeasurements_.deltaTij;
      const Vector3 biasAccIncr = bias.accelerometer() - preintegratedMeasurements_.biasHat.accelerometer();
      const Vector3 biasOmegaIncr = bias.gyroscope() - preintegratedMeasurements_.biasHat.gyroscope();

      Point3 pose_j_translation = pose_i.translation()
        + Point3(-pose_i.rotation().matrix() * (preintegratedMeasurements_.deltaPij
                 + preintegratedMeasurements_.delPdelBiasAcc * biasAccIncr
                 + preintegratedMeasurements_.delPdelBiasOmega * biasOmegaIncr)
                 - vel_i * deltaTij
                 + skewSymmetric(omegaCoriolis_) * vel_i * deltaTij*deltaTij  // Coriolis term - we got rid of the 2 wrt ins paper
                 - 0.5 * gravity_ * deltaTij*deltaTij);

      Vector3 vel_j_vector = vel_i
          - pose_i.rotation().matrix() * (preintegratedMeasurements_.deltaVij
                                        + preintegratedMeasurements_.delVdelBiasAcc * biasAccIncr
                                        + preintegratedMeasurements_.delVdelBiasOmega * biasOmegaIncr)
          + 2 * skewSymmetric(omegaCoriolis_) * vel_i * deltaTij  // Coriolis term
          - gravity_ * deltaTij;

      const Rot3 deltaRij_biascorrected = preintegratedMeasurements_.deltaRij.retract(preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr, Rot3::EXPMAP);
      Rot3 pose_j_rotation = pose_i.rotation().compose(deltaRij_biascorrected);

      pose_j = Pose3(pose_j_rotation, pose_j_translation);
      vel_j = LieVector(vel_j_vector);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const Pose3& pose_i, const LieVector& vel_i, const Pose3& pose_j, const LieVector& vel_j,
      const imuBias::ConstantBias& bias,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none,
        boost::optional<Matrix&> H5 = boost::none) const
    {

      const double& deltaTij = preintegratedMeasurements_.deltaTij;
      const Vector3 biasAccIncr = bias.accelerometer() - preintegratedMeasurements_.biasHat.accelerometer();
      const Vector3 biasOmegaIncr = bias.gyroscope() - preintegratedMeasurements_.biasHat.gyroscope();


      const Rot3 deltaRij_biascorrected = preintegratedMeasurements_.deltaRij.retract(preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr, Rot3::EXPMAP);
      // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)

      Vector3 theta_biascorrected_corioliscorrected = Rot3::Logmap(deltaRij_biascorrected)  -
          pose_i.rotation().inverse().matrix() * omegaCoriolis_ * deltaTij; // Coriolis term

      const Rot3 deltaRij_biascorrected_corioliscorrected =
          Rot3::Expmap( theta_biascorrected_corioliscorrected );  // Coriolis term

      const Rot3 fRhat =
           deltaRij_biascorrected_corioliscorrected.between(pose_i.rotation().between(pose_j.rotation()));

      // Coriolis term
      const Matrix3 Jr1 = rightJacobianExpMapSO3(theta_biascorrected_corioliscorrected);

      const Matrix3 Jtheta = -Jr1 * skewSymmetric(pose_i.rotation().inverse().matrix() * omegaCoriolis_ * deltaTij);  // Coriolis term

      if(H1) {
        H1->resize(9,6);
        (*H1) <<
            // dfP/dRi
            pose_i.rotation().matrix() * skewSymmetric(preintegratedMeasurements_.deltaPij)
             + pose_i.rotation().matrix() * skewSymmetric(preintegratedMeasurements_.delPdelBiasOmega * biasOmegaIncr)
             + pose_i.rotation().matrix() * skewSymmetric(preintegratedMeasurements_.delPdelBiasAcc * biasAccIncr),
            // dfP/dPi
             - pose_i.rotation().matrix(),
            // dfV/dRi
            pose_i.rotation().matrix() * skewSymmetric(preintegratedMeasurements_.deltaVij)
            + pose_i.rotation().matrix() * skewSymmetric(preintegratedMeasurements_.delVdelBiasOmega * biasOmegaIncr)
            + pose_i.rotation().matrix() * skewSymmetric(preintegratedMeasurements_.delVdelBiasAcc * biasAccIncr),
            // dfV/dPi
            Matrix3::Zero(),
            // dfR/dRi
            - pose_j.rotation().between(pose_i.rotation()).matrix()
            - fRhat.inverse().matrix() * Jtheta,
            // dfR/dPi
            Matrix3::Zero();
      }
      if(H2) {
        H2->resize(9,3);
        (*H2) <<
            // dfP/dVi
            - Matrix3::Identity() * deltaTij
            + skewSymmetric(omegaCoriolis_) * deltaTij * deltaTij,  // Coriolis term - we got rid of the 2 wrt ins paper
            // dfV/dVi
            - Matrix3::Identity()
            + 2 * skewSymmetric(omegaCoriolis_) * deltaTij, // Coriolis term
            // dfR/dVi
            Matrix3::Zero();

      }
      if(H3) {

        H3->resize(9,6);
        (*H3) <<
            // dfP/dPosej
            Matrix3::Zero(), pose_j.rotation().matrix(),
            // dfV/dPosej
            Matrix::Zero(3,6),
            // dfR/dPosej
            Matrix3::Identity() , Matrix3::Zero();
      }
      if(H4) {
        H4->resize(9,3);
        (*H4) <<
            // dfP/dVj
            Matrix3::Zero(),
            // dfV/dVj
            Matrix3::Identity(),
            // dfR/dVj
            Matrix3::Zero();
      }
      if(H5) {

        // Coriolis term
        const Matrix3 Jrinv2 = rightJacobianExpMapSO3inverse(Rot3::Logmap(deltaRij_biascorrected));

        // Coriolis term
        const Matrix3 Jr3 = rightJacobianExpMapSO3(preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr);

        const Matrix3 JbiasOmega = Jr1 * Jrinv2 * Jr3 * preintegratedMeasurements_.delRdelBiasOmega;  // Coriolis term

        H5->resize(9,6);
        (*H5) <<
            // dfP/dBias
            - pose_i.rotation().matrix() * preintegratedMeasurements_.delPdelBiasAcc,
            - pose_i.rotation().matrix() * preintegratedMeasurements_.delPdelBiasOmega,
            // dfV/dBias
            - pose_i.rotation().matrix() * preintegratedMeasurements_.delVdelBiasAcc,
            - pose_i.rotation().matrix() * preintegratedMeasurements_.delVdelBiasOmega,
            // dfR/dBias
            Matrix::Zero(3,3),
            - fRhat.inverse().matrix() * JbiasOmega;
      }


      const Vector3 fp =
        pose_j.translation().vector() - pose_i.translation().vector()
        - pose_i.rotation().matrix() * (preintegratedMeasurements_.deltaPij
          + preintegratedMeasurements_.delPdelBiasAcc * biasAccIncr
          + preintegratedMeasurements_.delPdelBiasOmega * biasOmegaIncr)
        - vel_i * deltaTij
        + skewSymmetric(omegaCoriolis_) * vel_i * deltaTij*deltaTij  // Coriolis term - we got rid of the 2 wrt ins paper
        - 0.5 * gravity_ * deltaTij*deltaTij;



      const Vector3 fv =
        vel_j - vel_i - pose_i.rotation().matrix() * (preintegratedMeasurements_.deltaVij
          + preintegratedMeasurements_.delVdelBiasAcc * biasAccIncr
          + preintegratedMeasurements_.delVdelBiasOmega * biasOmegaIncr)
          + 2 * skewSymmetric(omegaCoriolis_) * vel_i * deltaTij  // Coriolis term
          - gravity_ * deltaTij;

      const Vector3 fR = Rot3::Logmap(fRhat);

      Vector r(9); r << fp, fv, fR;

      return r;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor5",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(preintegratedMeasurements_);
      ar & BOOST_SERIALIZATION_NVP(gravity_);
      ar & BOOST_SERIALIZATION_NVP(omegaCoriolis_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
    }
  }; // \class ImuFactor

  /// Typedef for Matlab interface
  typedef ImuFactor::PreintegratedMeasurements ImuFactorPreintegratedMeasurements;

} /// namespace gtsam

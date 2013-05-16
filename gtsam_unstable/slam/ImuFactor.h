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
  class ImuFactor: public NoiseModelFactor5<Pose3,LieVector,Pose3,LieVector,LieVector> {

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
      Vector3 deltaPij; ///< Preintegrated relative position (in frame i)
      Vector3 deltaVij; ///< Preintegrated relative velocity (in global frame)
      Rot3 deltaRij; ///< Preintegrated relative orientation (in frame i)
      double deltaTij; ///< Time interval from i to j

      Matrix3 delPdelBiasAcc; ///< Jacobian of preintegrated position w.r.t. acceleration bias
      Matrix3 delPdelBiasOmega; ///< Jacobian of preintegrated position w.r.t. angular rate bias
      Matrix3 delVdelBiasAcc; ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
      Matrix3 delVdelBiasOmega; ///< Jacobian of preintegrated velocity w.r.t. angular rate bias
      Matrix3 delRdelBiasOmega; ///< Jacobian of preintegrated rotation w.r.t. angular rate bias

      Vector3 biasAccHat; ///< Linear acceleration bias
      Vector3 biasOmegaHat; ///< Rotation rate bias

      Matrix measurementCovariance; ///< Covariance of the vector [measuredAcc measuredOmega integrationError] in R^(9X9)
      Matrix preintegratedMeasurementsCovariance; ///< Covariance of the preintegrated measurements in R^(9X9)


      /** Default constructor, initialize with no IMU measurements */
      PreintegratedMeasurements(
          Vector3 _biasAcc, ///< Current estimate of acceleration bias
          Vector3 _biasOmega, ///< Current estimate of rotation rate bias
          Matrix3 _measuredAccCovariance, ///< Covariance matrix of measuredAcc
          Matrix3 _measuredOmegaCovariance, ///< Covariance matrix of measuredAcc
          Matrix3 _integrationErrorCovariance ///< Covariance matrix of measuredAcc
      ) : deltaPij(Vector3::Zero()), deltaVij(Vector3::Zero()), deltaTij(0.0),
      delPdelBiasAcc(Matrix3::Zero()), delPdelBiasOmega(Matrix3::Zero()),
      delVdelBiasAcc(Matrix3::Zero()), delVdelBiasOmega(Matrix3::Zero()),
      delRdelBiasOmega(Matrix3::Zero()),
      biasAccHat(_biasAcc), biasOmegaHat(_biasOmega), measurementCovariance(9,9), preintegratedMeasurementsCovariance(9,9) {
        measurementCovariance << _measuredAccCovariance , Matrix3::Zero(), Matrix3::Zero(),
            Matrix3::Zero(), _measuredOmegaCovariance,  Matrix3::Zero(),
            Matrix3::Zero(),   Matrix3::Zero(),   _integrationErrorCovariance;
      }

      /** Add a single IMU measurement to the preintegration. */
      void integrateMeasurement(
          const Vector3& measuredAcc, ///< Measured linear acceleration (in body frame)
          const Vector3& measuredOmega, ///< Measured angular velocity (in body frame)
          double deltaT ///< Time step
      ) {
        // NOTE: order is important here because each update uses old values.

        const Rot3 Rincr = Rot3::Expmap((measuredOmega - biasOmegaHat) * deltaT);

        // Update Jacobians
        delPdelBiasAcc += delVdelBiasAcc * deltaT - 0.5 * deltaRij.matrix() * deltaT*deltaT;
        delPdelBiasOmega += delVdelBiasOmega * deltaT - 0.5 * deltaRij.matrix()
          * skewSymmetric(measuredAcc - biasAccHat) * deltaT*deltaT * delRdelBiasOmega;
        delVdelBiasAcc += -deltaRij.matrix() * deltaT;
        delVdelBiasOmega += -deltaRij.matrix() * skewSymmetric(measuredAcc - biasAccHat) * deltaT * delRdelBiasOmega;

        const Vector3 x = (measuredOmega - biasOmegaHat) * deltaT; // parametrization of so(3)
        const Matrix3 X = skewSymmetric(x); // element of Lie algebra so(3): X = x^
        double normx = norm_2(x);
        const Matrix3 Jr = Matrix3::Identity() - ((1-cos(normx))/(normx*normx)) * X +
            ((normx-sin(normx))/(normx*normx*normx)) * X * X;

        delRdelBiasOmega = Rincr.inverse().matrix() * delRdelBiasOmega - Jr  * deltaT;

        // Compute Preintegrated measurement covariance (this is not recursive, but we use the Jacobians computed before)
        Matrix Jpreintegrated(9,9);
                Jpreintegrated <<
                    //deltaP VS (measuredAcc   measuredOmega   intError)
                    - delPdelBiasAcc,  - delPdelBiasOmega, - delPdelBiasAcc,
                        //deltaV VS (measuredAcc   measuredOmega   intError)
                    - delVdelBiasAcc,  - delVdelBiasOmega, Matrix3::Zero(),
                         //deltaR VS (measuredAcc   measuredOmega   intError)
                         Matrix3::Zero(), -delRdelBiasOmega, Matrix3::Zero();

        preintegratedMeasurementsCovariance = Jpreintegrated * measurementCovariance * Jpreintegrated.transpose();


        // Update preintegrated measurements
        deltaPij += deltaVij * deltaT + 0.5 * deltaRij.matrix() * (measuredAcc - biasAccHat) * deltaT*deltaT;
        deltaVij += deltaRij.matrix() * (measuredAcc - biasAccHat) * deltaT;
        deltaRij = deltaRij * Rincr;
        deltaTij += deltaT;

      }
    };

  private:

    typedef ImuFactor This;
    typedef NoiseModelFactor5<Pose3,LieVector,Pose3,LieVector,LieVector> Base;

    PreintegratedMeasurements preintegratedMeasurements_;
    Vector3 gravity_;
    Vector3 omegaCoriolis_;

  public:

    /** Shorthand for a smart pointer to a factor */
    typedef typename boost::shared_ptr<ImuFactor> shared_ptr;

    /** Default constructor - only use for serialization */
    ImuFactor() : preintegratedMeasurements_(Vector3::Zero(), Vector3::Zero(),  Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero()) {}

    /** Constructor */
    ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
        const PreintegratedMeasurements& preintegratedMeasurements, const Vector3& gravity, const Vector3& omegaCoriolis,
        const SharedNoiseModel& model) :
      Base(model, pose_i, vel_i, pose_j, vel_j, bias),
      preintegratedMeasurements_(preintegratedMeasurements),
      gravity_(gravity),
      omegaCoriolis_(omegaCoriolis) {
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
          << keyFormatter(this->key4()) << ")\n";
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol);
    }

    /** Access the preintegrated measurements. */
    const PreintegratedMeasurements& preintegratedMeasurements() const {
      return preintegratedMeasurements_; }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const Pose3& pose_i, const LieVector& vel_i, const Pose3& pose_j, const LieVector& vel_j,
      const LieVector& bias,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none,
        boost::optional<Matrix&> H5 = boost::none) const
    {

      const double& deltaTij = preintegratedMeasurements_.deltaTij;
      const Vector3 biasAccIncr = bias.head(3) - preintegratedMeasurements_.biasAccHat;
      const Vector3 biasOmegaIncr = bias.tail(3) - preintegratedMeasurements_.biasOmegaHat;


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
      ar & boost::serialization::make_nvp("NoiseModelFactor4",
          boost::serialization::base_object<Base>(*this));
    }
  }; // \class ImuFactor

} /// namespace gtsam

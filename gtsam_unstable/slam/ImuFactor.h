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

/* External or standard includes */
#include <ostream>


namespace gtsam {

  /**
   * 
   * @addtogroup SLAM
   */
  class ImuFactor: public NoiseModelFactor6<Pose3,LieVector,Pose3,LieVector,LieVector,LieVector> {

  public:

    /** Struct to store results of preintegrating IMU measurements.  Can be build
     * incrementally so as to avoid costly integration at time of factor
     * construction. */
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

      /** Default constructor, initialize with no IMU measurements */
      PreintegratedMeasurements(
          Vector3 _biasAcc, ///< Current estimate of acceleration bias
          Vector3 _biasOmega ///< Current estimate of rotation rate bias
      ) : deltaPij(Vector3::Zero()), deltaVij(Vector3::Zero()), deltaTij(0.0),
      biasAccHat(_biasAcc), biasOmegaHat(_biasOmega),
      delPdelBiasAcc(Matrix3::Zero()), delPdelBiasOmega(Matrix3::Zero()),
      delVdelBiasAcc(Matrix3::Zero()), delVdelBiasOmega(Matrix3::Zero()),
      delRdelBiasOmega(Matrix3::Zero()) {}

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
        delPdelBiasOmega += delVdelBiasOmega * deltaT + 0.5 * deltaRij.matrix()
          * skewSymmetric(measuredAcc - biasAccHat) * deltaT*deltaT * delRdelBiasOmega;
        delVdelBiasAcc += -deltaRij.matrix() * deltaT;
        delVdelBiasOmega += -deltaRij.matrix() * skewSymmetric(measuredAcc - biasAccHat) * deltaT * delRdelBiasOmega;
        delRdelBiasOmega = -Rincr.matrix() * delRdelBiasOmega - Matrix3::Identity() * deltaT;

        // Update preintegrated measurements
        deltaPij += deltaVij * deltaT + 0.5 * deltaRij.matrix() * (measuredAcc - biasAccHat) * deltaT*deltaT;
        deltaVij += deltaRij.matrix() * (measuredAcc - biasAccHat) * deltaT;
        deltaRij = deltaRij * Rincr;
        deltaTij += deltaT;
      }
    };

  private:

    typedef ImuFactor This;
    typedef NoiseModelFactor6<Pose3,LieVector,Pose3,LieVector,LieVector,LieVector> Base;

    PreintegratedMeasurements preintegratedMeasurements_;
    Vector3 gravity_;

  public:

    /** Shorthand for a smart pointer to a factor */
    typedef typename boost::shared_ptr<ImuFactor> shared_ptr;

    /** Default constructor - only use for serialization */
    ImuFactor() : preintegratedMeasurements_(Vector3::Zero(), Vector3::Zero()) {}

    /** Constructor */
    ImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j,
        const PreintegratedMeasurements& preintegratedMeasurements, const Vector3& gravity,
        const SharedNoiseModel& model) :
      Base(model, pose_i, vel_i, pose_j, vel_j),
      preintegratedMeasurements_(preintegratedMeasurements),
      gravity_(gravity) {
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

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const Pose3& pose_i, const LieVector& vel_i, const Pose3& pose_j, const LieVector& vel_j,
      const LieVector& biasAcc, const LieVector& biasOmega,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none,
        boost::optional<Matrix&> H5 = boost::none,
        boost::optional<Matrix&> H6 = boost::none) const
    {

      const double& deltaTij = preintegratedMeasurements_.deltaTij;

      if(H1) {
        H1->resize(9,6);
        (*H1) << pose_i.rotation().matrix() * skewSymmetric(preintegratedMeasurements_.deltaPij), - pose_i.rotation().matrix(),
            pose_i.rotation().matrix() * skewSymmetric(preintegratedMeasurements_.deltaVij), Matrix3::Zero(),
            -pose_j.rotation().between(pose_i.rotation()).matrix() , Matrix3::Zero();
      }
      if(H2) {
        H2->resize(9,3);
        (*H2) <<  - Matrix3::Identity() * deltaTij,
            - Matrix3::Identity(),
            Matrix3::Zero();

      }
      if(H3) {
        H3->resize(9,6);
        (*H3) << Matrix3::Zero(), pose_j.rotation().matrix(),
            Matrix::Zero(3,6),
            Matrix3::Identity() , Matrix3::Zero();
      }
      if(H4) {
        H4->resize(9,3);
        (*H4) << Matrix3::Zero(),
            Matrix3::Identity(),
            Matrix3::Zero();
      }

      const Vector3 biasAccIncr = biasAcc - preintegratedMeasurements_.biasAccHat;
      const Vector3 biasOmegaIncr = biasOmega - preintegratedMeasurements_.biasOmegaHat;

      const Vector3 fp =
        pose_j.translation().vector() - pose_i.translation().vector()
        - pose_i.rotation().matrix() * (preintegratedMeasurements_.deltaPij
          + preintegratedMeasurements_.delPdelBiasAcc * biasAccIncr
          + preintegratedMeasurements_.delPdelBiasOmega * biasOmegaIncr)
        - vel_i * deltaTij
        - 0.5 * gravity_ * deltaTij*deltaTij;

      const Vector3 fv =
        vel_j - vel_i - pose_i.rotation().matrix() * (preintegratedMeasurements_.deltaVij
          + preintegratedMeasurements_.delVdelBiasAcc * biasAccIncr
          + preintegratedMeasurements_.delVdelBiasOmega * biasOmegaIncr)
        - gravity_ * deltaTij;

      const Vector3 fR =
        preintegratedMeasurements_.deltaRij.retract(preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr)
        .localCoordinates(pose_i.rotation().between(pose_j.rotation()));

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

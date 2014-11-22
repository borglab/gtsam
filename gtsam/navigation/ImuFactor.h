/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ImuFactor.h
 *  @author Luca Carlone, Stephen Williams, Richard Roberts, Vadim Indelman, David Jensen
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
   *
   * If you are using the factor, please cite:
   * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating conditionally
   * independent sets in factor graphs: a unifying perspective based on smart factors,
   * Int. Conf. on Robotics and Automation (ICRA), 2014.
   *
   ** REFERENCES:
   * [1] G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
   * [2] T. Lupton and S.Sukkarieh, "Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built
   * Environments Without Initial Conditions", TRO, 28(1):61-76, 2012.
   * [3] L. Carlone, S. Williams, R. Roberts, "Preintegrated IMU factor: Computation of the Jacobian Matrices", Tech. Report, 2013.
   */

  class ImuFactor: public NoiseModelFactor5<Pose3,LieVector,Pose3,LieVector,imuBias::ConstantBias> {

  public:

    /** Struct to store results of preintegrating IMU measurements.  Can be build
     * incrementally so as to avoid costly integration at time of factor construction. */

    /** CombinedPreintegratedMeasurements accumulates (integrates) the IMU measurements (rotation rates and accelerations)
         * and the corresponding covariance matrix. The measurements are then used to build the Preintegrated IMU factor*/
    class PreintegratedMeasurements {
    public:
      imuBias::ConstantBias biasHat; ///< Acceleration and angular rate bias values used during preintegration
      Matrix measurementCovariance; ///< (Raw measurements uncertainty) Covariance of the vector [integrationError measuredAcc measuredOmega] in R^(9X9)

      Vector3 deltaPij; ///< Preintegrated relative position (does not take into account velocity at time i, see deltap+, in [2]) (in frame i)
      Vector3 deltaVij; ///< Preintegrated relative velocity (in global frame)
      Rot3 deltaRij; ///< Preintegrated relative orientation (in frame i)
      double deltaTij; ///< Time interval from i to j

      Matrix3 delPdelBiasAcc; ///< Jacobian of preintegrated position w.r.t. acceleration bias
      Matrix3 delPdelBiasOmega; ///< Jacobian of preintegrated position w.r.t. angular rate bias
      Matrix3 delVdelBiasAcc; ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
      Matrix3 delVdelBiasOmega; ///< Jacobian of preintegrated velocity w.r.t. angular rate bias
      Matrix3 delRdelBiasOmega; ///< Jacobian of preintegrated rotation w.r.t. angular rate bias
      Matrix PreintMeasCov; ///< Covariance matrix of the preintegrated measurements (first-order propagation from *measurementCovariance*)
      bool use2ndOrderIntegration_; ///< Controls the order of integration

      /** Default constructor, initialize with no IMU measurements */
      PreintegratedMeasurements(
          const imuBias::ConstantBias& bias, ///< Current estimate of acceleration and rotation rate biases
          const Matrix3& measuredAccCovariance, ///< Covariance matrix of measuredAcc
          const Matrix3& measuredOmegaCovariance, ///< Covariance matrix of measuredAcc
          const Matrix3& integrationErrorCovariance, ///< Covariance matrix of measuredAcc
          const bool use2ndOrderIntegration = false ///< Controls the order of integration
      ) : biasHat(bias), measurementCovariance(9,9), deltaPij(Vector3::Zero()), deltaVij(Vector3::Zero()), deltaTij(0.0),
      delPdelBiasAcc(Matrix3::Zero()), delPdelBiasOmega(Matrix3::Zero()),
      delVdelBiasAcc(Matrix3::Zero()), delVdelBiasOmega(Matrix3::Zero()),
      delRdelBiasOmega(Matrix3::Zero()), PreintMeasCov(9,9), use2ndOrderIntegration_(use2ndOrderIntegration)
      {
        measurementCovariance << integrationErrorCovariance , Matrix3::Zero(), Matrix3::Zero(),
                                       Matrix3::Zero(), measuredAccCovariance,  Matrix3::Zero(),
                                       Matrix3::Zero(),   Matrix3::Zero(), measuredOmegaCovariance;
        PreintMeasCov = Matrix::Zero(9,9);
      }

      PreintegratedMeasurements() :
      biasHat(imuBias::ConstantBias()), measurementCovariance(9,9), deltaPij(Vector3::Zero()), deltaVij(Vector3::Zero()), deltaTij(0.0),
      delPdelBiasAcc(Matrix3::Zero()), delPdelBiasOmega(Matrix3::Zero()),
      delVdelBiasAcc(Matrix3::Zero()), delVdelBiasOmega(Matrix3::Zero()),
      delRdelBiasOmega(Matrix3::Zero()), PreintMeasCov(9,9)
      {
          measurementCovariance =  Matrix::Zero(9,9);
          PreintMeasCov = Matrix::Zero(9,9);
      }

      /** print */
      void print(const std::string& s = "Preintegrated Measurements:") const {
        std::cout << s << std::endl;
        biasHat.print("  biasHat");
        std::cout << "  deltaTij " << deltaTij << std::endl;
        std::cout << "  deltaPij [ " << deltaPij.transpose() << " ]" << std::endl;
        std::cout << "  deltaVij [ " << deltaVij.transpose() << " ]" << std::endl;
        deltaRij.print("  deltaRij ");
        std::cout << "  measurementCovariance [ " << measurementCovariance << " ]" << std::endl;
        std::cout << "  PreintMeasCov [ " << PreintMeasCov << " ]" << std::endl;
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

      void resetIntegration(){
        deltaPij = Vector3::Zero();
        deltaVij = Vector3::Zero();
        deltaRij = Rot3();
        deltaTij = 0.0;
        delPdelBiasAcc = Matrix3::Zero();
        delPdelBiasOmega = Matrix3::Zero();
        delVdelBiasAcc = Matrix3::Zero();
        delVdelBiasOmega = Matrix3::Zero();
        delRdelBiasOmega = Matrix3::Zero();
        PreintMeasCov = Matrix::Zero(9,9);
      }

      /** Add a single IMU measurement to the preintegration. */
      void integrateMeasurement(
          const Vector3& measuredAcc, ///< Measured linear acceleration (in body frame)
          const Vector3& measuredOmega, ///< Measured angular velocity (in body frame)
          double deltaT, ///< Time step
          boost::optional<const Pose3&> body_P_sensor = boost::none ///< Sensor frame
      ) {

        // NOTE: order is important here because each update uses old values.
        // First we compensate the measurements for the bias
        Vector3 correctedAcc = biasHat.correctAccelerometer(measuredAcc);
        Vector3 correctedOmega = biasHat.correctGyroscope(measuredOmega);

        // Then compensate for sensor-body displacement: we express the quantities (originally in the IMU frame) into the body frame
        if(body_P_sensor){
          Matrix3 body_R_sensor = body_P_sensor->rotation().matrix();
          correctedOmega = body_R_sensor * correctedOmega; // rotation rate vector in the body frame
          Matrix3 body_omega_body__cross = skewSymmetric(correctedOmega);
          correctedAcc = body_R_sensor * correctedAcc - body_omega_body__cross * body_omega_body__cross * body_P_sensor->translation().vector();
          // linear acceleration vector in the body frame
        }

        const Vector3 theta_incr = correctedOmega * deltaT; // rotation vector describing rotation increment computed from the current rotation rate measurement
        const Rot3 Rincr = Rot3::Expmap(theta_incr); // rotation increment computed from the current rotation rate measurement

        const Matrix3 Jr_theta_incr = Rot3::rightJacobianExpMapSO3(theta_incr); // Right jacobian computed at theta_incr

        // Update Jacobians
        /* ----------------------------------------------------------------------------------------------------------------------- */
        if(!use2ndOrderIntegration_){
          delPdelBiasAcc += delVdelBiasAcc * deltaT;
          delPdelBiasOmega += delVdelBiasOmega * deltaT;
        }else{
          delPdelBiasAcc += delVdelBiasAcc * deltaT - 0.5 * deltaRij.matrix() * deltaT*deltaT;
          delPdelBiasOmega += delVdelBiasOmega * deltaT - 0.5 * deltaRij.matrix()
                                    * skewSymmetric(biasHat.correctAccelerometer(measuredAcc)) * deltaT*deltaT * delRdelBiasOmega;
        }
        delVdelBiasAcc += -deltaRij.matrix() * deltaT;
        delVdelBiasOmega += -deltaRij.matrix() * skewSymmetric(correctedAcc) * deltaT * delRdelBiasOmega;
        delRdelBiasOmega = Rincr.inverse().matrix() * delRdelBiasOmega - Jr_theta_incr  * deltaT;

        // Update preintegrated measurements covariance
        /* ----------------------------------------------------------------------------------------------------------------------- */
        Matrix3 Z_3x3 = Matrix3::Zero();
        Matrix3 I_3x3 = Matrix3::Identity();
        const Vector3 theta_i = Rot3::Logmap(deltaRij); // parametrization of so(3)
        const Matrix3 Jr_theta_i = Rot3::rightJacobianExpMapSO3(theta_i);

        Rot3 Rot_j = deltaRij * Rincr;
        const Vector3 theta_j = Rot3::Logmap(Rot_j); // parametrization of so(3)
        const Matrix3 Jrinv_theta_j = Rot3::rightJacobianExpMapSO3inverse(theta_j);

        // Update preintegrated measurements covariance: as in [2] we consider a first order propagation that
        // can be seen as a prediction phase in an EKF framework
        Matrix H_pos_pos    = I_3x3;
        Matrix H_pos_vel    = I_3x3 * deltaT;
        Matrix H_pos_angles = Z_3x3;

        Matrix H_vel_pos    = Z_3x3;
        Matrix H_vel_vel    = I_3x3;
        Matrix H_vel_angles = - deltaRij.matrix() * skewSymmetric(correctedAcc) * Jr_theta_i * deltaT;
        // analytic expression corresponding to the following numerical derivative
        // Matrix H_vel_angles = numericalDerivative11<LieVector, LieVector>(boost::bind(&PreIntegrateIMUObservations_delta_vel, correctedOmega, correctedAcc, deltaT, _1, deltaVij), theta_i);

        Matrix H_angles_pos   = Z_3x3;
        Matrix H_angles_vel    = Z_3x3;
        Matrix H_angles_angles = Jrinv_theta_j * Rincr.inverse().matrix() * Jr_theta_i;
        // analytic expression corresponding to the following numerical derivative
        // Matrix H_angles_angles = numericalDerivative11<LieVector, LieVector>(boost::bind(&PreIntegrateIMUObservations_delta_angles, correctedOmega, deltaT, _1), thetaij);

        // overall Jacobian wrt preintegrated measurements (df/dx)
        Matrix F(9,9);
        F << H_pos_pos, H_pos_vel,  H_pos_angles,
            H_vel_pos, H_vel_vel, H_vel_angles,
            H_angles_pos, H_angles_vel, H_angles_angles;


        // first order uncertainty propagation
        // the deltaT allows to pass from continuous time noise to discrete time noise
        PreintMeasCov = F * PreintMeasCov * F.transpose() + measurementCovariance * deltaT ;

        // Update preintegrated measurements
        /* ----------------------------------------------------------------------------------------------------------------------- */
        if(!use2ndOrderIntegration_){
          deltaPij += deltaVij * deltaT;
        }else{
          deltaPij += deltaVij * deltaT + 0.5 * deltaRij.matrix() * biasHat.correctAccelerometer(measuredAcc) * deltaT*deltaT;
        }
        deltaVij += deltaRij.matrix() * correctedAcc * deltaT;
        deltaRij = deltaRij * Rincr;
        deltaTij += deltaT;
      }

      /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
      // This function is only used for test purposes (compare numerical derivatives wrt analytic ones)
      static inline Vector PreIntegrateIMUObservations_delta_vel(const Vector& msr_gyro_t, const Vector& msr_acc_t, const double msr_dt,
              const Vector3& delta_angles, const Vector& delta_vel_in_t0){

          // Note: all delta terms refer to an IMU\sensor system at t0

        Vector body_t_a_body = msr_acc_t;
        Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);

          return delta_vel_in_t0 + R_t_to_t0.matrix() * body_t_a_body * msr_dt;
      }

      // This function is only used for test purposes (compare numerical derivatives wrt analytic ones)
      static inline Vector PreIntegrateIMUObservations_delta_angles(const Vector& msr_gyro_t, const double msr_dt,
              const Vector3& delta_angles){

          // Note: all delta terms refer to an IMU\sensor system at t0

          // Calculate the corrected measurements using the Bias object
        Vector body_t_omega_body= msr_gyro_t;

          Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);

          R_t_to_t0    = R_t_to_t0 * Rot3::Expmap( body_t_omega_body*msr_dt );
          return Rot3::Logmap(R_t_to_t0);
      }
      /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

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

    bool use2ndOrderCoriolis_; ///< Controls whether higher order terms are included when calculating the Coriolis Effect

  public:

    /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
    typedef typename boost::shared_ptr<ImuFactor> shared_ptr;
#else
    typedef boost::shared_ptr<ImuFactor> shared_ptr;
#endif

    /** Default constructor - only use for serialization */
    ImuFactor() : preintegratedMeasurements_(imuBias::ConstantBias(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero()) {}

    /** Constructor */
    ImuFactor(
      Key pose_i, ///< previous pose key
      Key vel_i, ///< previous velocity key
      Key pose_j, ///< current pose key
      Key vel_j, ///< current velocity key
      Key bias, ///< previous bias key
        const PreintegratedMeasurements& preintegratedMeasurements, ///< preintegrated IMU measurements
        const Vector3& gravity, ///< gravity vector
        const Vector3& omegaCoriolis, ///< rotation rate of the inertial frame
        boost::optional<const Pose3&> body_P_sensor = boost::none, ///< The Pose of the sensor frame in the body frame
        const bool use2ndOrderCoriolis = false ///< When true, the second-order term is used in the calculation of the Coriolis Effect
    ) :
      Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.PreintMeasCov), pose_i, vel_i, pose_j, vel_j, bias),
      preintegratedMeasurements_(preintegratedMeasurements),
      gravity_(gravity),
      omegaCoriolis_(omegaCoriolis),
      body_P_sensor_(body_P_sensor),
      use2ndOrderCoriolis_(use2ndOrderCoriolis){
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
          && preintegratedMeasurements_.equals(e->preintegratedMeasurements_, tol)
          && equal_with_abs_tol(gravity_, e->gravity_, tol)
          && equal_with_abs_tol(omegaCoriolis_, e->omegaCoriolis_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /** Access the preintegrated measurements. */
    const PreintegratedMeasurements& preintegratedMeasurements() const {
      return preintegratedMeasurements_; }

    const Vector3& gravity() const { return gravity_; }

    const Vector3& omegaCoriolis() const { return omegaCoriolis_; }

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

      // we give some shorter name to rotations and translations
      const Rot3 Rot_i = pose_i.rotation();
      const Rot3 Rot_j = pose_j.rotation();
      const Vector3 pos_i = pose_i.translation().vector();
      const Vector3 pos_j = pose_j.translation().vector();

      // We compute factor's Jacobians
      /* ---------------------------------------------------------------------------------------------------- */
      const Rot3 deltaRij_biascorrected = preintegratedMeasurements_.deltaRij.retract(preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr, Rot3::EXPMAP);
      // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)

      Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);

      Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected  -
          Rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij; // Coriolis term

      const Rot3 deltaRij_biascorrected_corioliscorrected =
          Rot3::Expmap( theta_biascorrected_corioliscorrected );

      const Rot3 fRhat = deltaRij_biascorrected_corioliscorrected.between(Rot_i.between(Rot_j));

      const Matrix3 Jr_theta_bcc = Rot3::rightJacobianExpMapSO3(theta_biascorrected_corioliscorrected);

      const Matrix3 Jtheta = -Jr_theta_bcc  * skewSymmetric(Rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij);

      const Matrix3 Jrinv_fRhat = Rot3::rightJacobianExpMapSO3inverse(Rot3::Logmap(fRhat));

      if(H1) {
        H1->resize(9,6);

        Matrix3 dfPdPi;
        Matrix3 dfVdPi;
        if(use2ndOrderCoriolis_){
          dfPdPi = - Rot_i.matrix() + 0.5 * skewSymmetric(omegaCoriolis_) * skewSymmetric(omegaCoriolis_) * Rot_i.matrix() * deltaTij*deltaTij;
          dfVdPi = skewSymmetric(omegaCoriolis_) * skewSymmetric(omegaCoriolis_) * Rot_i.matrix() * deltaTij;
        }
        else{
          dfPdPi = - Rot_i.matrix();
          dfVdPi = Matrix3::Zero();
        }

    (*H1) <<
      // dfP/dRi
      Rot_i.matrix() * skewSymmetric(preintegratedMeasurements_.deltaPij
        + preintegratedMeasurements_.delPdelBiasOmega * biasOmegaIncr + preintegratedMeasurements_.delPdelBiasAcc * biasAccIncr),
      // dfP/dPi
      dfPdPi,
      // dfV/dRi
      Rot_i.matrix() * skewSymmetric(preintegratedMeasurements_.deltaVij
        + preintegratedMeasurements_.delVdelBiasOmega * biasOmegaIncr + preintegratedMeasurements_.delVdelBiasAcc * biasAccIncr),
      // dfV/dPi
      dfVdPi,
      // dfR/dRi
      Jrinv_fRhat *  (- Rot_j.between(Rot_i).matrix() - fRhat.inverse().matrix() * Jtheta),
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
            Matrix3::Zero(), Rot_j.matrix(),
            // dfV/dPosej
            Matrix::Zero(3,6),
            // dfR/dPosej
            Jrinv_fRhat *  ( Matrix3::Identity() ), Matrix3::Zero();
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

        const Matrix3 Jrinv_theta_bc = Rot3::rightJacobianExpMapSO3inverse(theta_biascorrected);
        const Matrix3 Jr_JbiasOmegaIncr = Rot3::rightJacobianExpMapSO3(preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr);
        const Matrix3 JbiasOmega = Jr_theta_bcc * Jrinv_theta_bc * Jr_JbiasOmegaIncr * preintegratedMeasurements_.delRdelBiasOmega;

        H5->resize(9,6);
        (*H5) <<
            // dfP/dBias
            - Rot_i.matrix() * preintegratedMeasurements_.delPdelBiasAcc,
            - Rot_i.matrix() * preintegratedMeasurements_.delPdelBiasOmega,
            // dfV/dBias
            - Rot_i.matrix() * preintegratedMeasurements_.delVdelBiasAcc,
            - Rot_i.matrix() * preintegratedMeasurements_.delVdelBiasOmega,
            // dfR/dBias
            Matrix::Zero(3,3),
            Jrinv_fRhat * ( - fRhat.inverse().matrix() * JbiasOmega);
      }

      // Evaluate residual error, according to [3]
      /* ---------------------------------------------------------------------------------------------------- */
      const Vector3 fp =
          pos_j - pos_i
          - Rot_i.matrix() * (preintegratedMeasurements_.deltaPij
              + preintegratedMeasurements_.delPdelBiasAcc * biasAccIncr
              + preintegratedMeasurements_.delPdelBiasOmega * biasOmegaIncr)
              - vel_i * deltaTij
              + skewSymmetric(omegaCoriolis_) * vel_i * deltaTij*deltaTij  // Coriolis term - we got rid of the 2 wrt ins paper
              - 0.5 * gravity_ * deltaTij*deltaTij;

      const Vector3 fv =
          vel_j - vel_i - Rot_i.matrix() * (preintegratedMeasurements_.deltaVij
              + preintegratedMeasurements_.delVdelBiasAcc * biasAccIncr
              + preintegratedMeasurements_.delVdelBiasOmega * biasOmegaIncr)
              + 2 * skewSymmetric(omegaCoriolis_) * vel_i * deltaTij  // Coriolis term
              - gravity_ * deltaTij;

      const Vector3 fR = Rot3::Logmap(fRhat);

      Vector r(9); r << fp, fv, fR;
      return r;
    }


    /** predicted states from IMU */
    static void Predict(const Pose3& pose_i, const LieVector& vel_i, Pose3& pose_j, LieVector& vel_j,
        const imuBias::ConstantBias& bias, const PreintegratedMeasurements preintegratedMeasurements,
        const Vector3& gravity, const Vector3& omegaCoriolis, boost::optional<const Pose3&> body_P_sensor = boost::none,
        const bool use2ndOrderCoriolis = false)
    {

      const double& deltaTij = preintegratedMeasurements.deltaTij;
      const Vector3 biasAccIncr = bias.accelerometer() - preintegratedMeasurements.biasHat.accelerometer();
      const Vector3 biasOmegaIncr = bias.gyroscope() - preintegratedMeasurements.biasHat.gyroscope();

      const Rot3 Rot_i = pose_i.rotation();
      const Vector3 pos_i = pose_i.translation().vector();

      // Predict state at time j
      /* ---------------------------------------------------------------------------------------------------- */
    Vector3 pos_j =  pos_i + Rot_i.matrix() * (preintegratedMeasurements.deltaPij
      + preintegratedMeasurements.delPdelBiasAcc * biasAccIncr
      + preintegratedMeasurements.delPdelBiasOmega * biasOmegaIncr)
      + vel_i * deltaTij
      - skewSymmetric(omegaCoriolis) * vel_i * deltaTij*deltaTij  // Coriolis term - we got rid of the 2 wrt ins paper
      + 0.5 * gravity * deltaTij*deltaTij;

    vel_j = LieVector(vel_i + Rot_i.matrix() * (preintegratedMeasurements.deltaVij
      + preintegratedMeasurements.delVdelBiasAcc * biasAccIncr
      + preintegratedMeasurements.delVdelBiasOmega * biasOmegaIncr)
      - 2 * skewSymmetric(omegaCoriolis) * vel_i * deltaTij  // Coriolis term
      + gravity * deltaTij);

      if(use2ndOrderCoriolis){
        pos_j += - 0.5 * skewSymmetric(omegaCoriolis) * skewSymmetric(omegaCoriolis) * pos_i * deltaTij*deltaTij;  // 2nd order coriolis term for position
        vel_j += - skewSymmetric(omegaCoriolis) * skewSymmetric(omegaCoriolis) * pos_i * deltaTij; // 2nd order term for velocity
      }

      const Rot3 deltaRij_biascorrected = preintegratedMeasurements.deltaRij.retract(preintegratedMeasurements.delRdelBiasOmega * biasOmegaIncr, Rot3::EXPMAP);
      // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)
      Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);
      Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected  -
          Rot_i.inverse().matrix() * omegaCoriolis * deltaTij; // Coriolis term
      const Rot3 deltaRij_biascorrected_corioliscorrected =
          Rot3::Expmap( theta_biascorrected_corioliscorrected );
      const Rot3 Rot_j = Rot_i.compose( deltaRij_biascorrected_corioliscorrected  );

      pose_j = Pose3( Rot_j, Point3(pos_j) );
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

  typedef ImuFactor::PreintegratedMeasurements ImuFactorPreintegratedMeasurements;

} /// namespace gtsam

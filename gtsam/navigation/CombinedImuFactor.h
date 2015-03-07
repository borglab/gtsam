/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  CombinedImuFactor.h
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
   * REFERENCES:
   * [1] G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
   * [2] T. Lupton and S.Sukkarieh, "Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built
   * Environments Without Initial Conditions", TRO, 28(1):61-76, 2012.
   * [3] L. Carlone, S. Williams, R. Roberts, "Preintegrated IMU factor: Computation of the Jacobian Matrices", Tech. Report, 2013.
   */

  class CombinedImuFactor: public NoiseModelFactor6<Pose3,LieVector,Pose3,LieVector,imuBias::ConstantBias,imuBias::ConstantBias> {

  public:

    /** Struct to store results of preintegrating IMU measurements.  Can be build
     * incrementally so as to avoid costly integration at time of factor construction. */

    /** CombinedPreintegratedMeasurements accumulates (integrates) the IMU measurements (rotation rates and accelerations)
     * and the corresponding covariance matrix. The measurements are then used to build the Preintegrated IMU factor*/
    class CombinedPreintegratedMeasurements {
    public:
      imuBias::ConstantBias biasHat; ///< Acceleration and angular rate bias values used during preintegration
      Matrix measurementCovariance; ///< (Raw measurements uncertainty) Covariance of the vector
      ///< [integrationError measuredAcc measuredOmega biasAccRandomWalk biasOmegaRandomWalk biasAccInit biasOmegaInit] in R^(21 x 21)

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

      ///< In the combined factor is also includes the biases and keeps the correlation between the preintegrated measurements and the biases
      ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION BiasAcc BiasOmega]
      /** Default constructor, initialize with no IMU measurements */
      CombinedPreintegratedMeasurements(
          const imuBias::ConstantBias& bias, ///< Current estimate of acceleration and rotation rate biases
          const Matrix3& measuredAccCovariance, ///< Covariance matrix of measuredAcc
          const Matrix3& measuredOmegaCovariance, ///< Covariance matrix of measuredAcc
          const Matrix3& integrationErrorCovariance, ///< Covariance matrix of measuredAcc
          const Matrix3& biasAccCovariance, ///< Covariance matrix of biasAcc (random walk describing BIAS evolution)
          const Matrix3& biasOmegaCovariance, ///< Covariance matrix of biasOmega (random walk describing BIAS evolution)
          const Matrix& biasAccOmegaInit, ///< Covariance of biasAcc & biasOmega when preintegrating measurements
          const bool use2ndOrderIntegration = false ///< Controls the order of integration
          ///< (this allows to consider the uncertainty of the BIAS choice when integrating the measurements)
      ) : biasHat(bias), measurementCovariance(21,21), deltaPij(Vector3::Zero()), deltaVij(Vector3::Zero()), deltaTij(0.0),
      delPdelBiasAcc(Matrix3::Zero()), delPdelBiasOmega(Matrix3::Zero()),
      delVdelBiasAcc(Matrix3::Zero()), delVdelBiasOmega(Matrix3::Zero()),
      delRdelBiasOmega(Matrix3::Zero()), PreintMeasCov(Matrix::Zero(15,15)),
      use2ndOrderIntegration_(use2ndOrderIntegration)
      {
          // COVARIANCE OF: [Integration AccMeasurement OmegaMeasurement BiasAccRandomWalk BiasOmegaRandomWalk (BiasAccInit BiasOmegaInit)] SIZE (21x21)
        measurementCovariance << integrationErrorCovariance , Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(),      Matrix3::Zero(),
                                       Matrix3::Zero(), measuredAccCovariance,  Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(),     Matrix3::Zero(),
                                       Matrix3::Zero(),   Matrix3::Zero(), measuredOmegaCovariance, Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(),  Matrix3::Zero(),
                                       Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), biasAccCovariance, Matrix3::Zero(),   Matrix3::Zero(),        Matrix3::Zero(),
                                       Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), biasOmegaCovariance, Matrix3::Zero(),        Matrix3::Zero(),
                                       Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(),     biasAccOmegaInit.block(0,0,3,3),    biasAccOmegaInit.block(0,3,3,3),
                                       Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(),     biasAccOmegaInit.block(3,0,3,3),        biasAccOmegaInit.block(3,3,3,3);

      }

      CombinedPreintegratedMeasurements() :
      biasHat(imuBias::ConstantBias()), measurementCovariance(21,21), deltaPij(Vector3::Zero()), deltaVij(Vector3::Zero()), deltaTij(0.0),
      delPdelBiasAcc(Matrix3::Zero()), delPdelBiasOmega(Matrix3::Zero()),
      delVdelBiasAcc(Matrix3::Zero()), delVdelBiasOmega(Matrix3::Zero()),
      delRdelBiasOmega(Matrix3::Zero()), PreintMeasCov(Matrix::Zero(15,15))
      {
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
      bool equals(const CombinedPreintegratedMeasurements& expected, double tol=1e-9) const {
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
        PreintMeasCov = Matrix::Zero(15,15);
      }

      /** Add a single IMU measurement to the preintegration. */
      void integrateMeasurement(
          const Vector3& measuredAcc, ///< Measured linear acceleration (in body frame)
          const Vector3& measuredOmega, ///< Measured angular velocity (in body frame)
          double deltaT, ///< Time step
          boost::optional<const Pose3&> body_P_sensor = boost::none ///< Sensor frame
      ) {
        // NOTE: order is important here because each update uses old values, e.g., velocity and position updates are based on previous rotation estimate.
        // First we compensate the measurements for the bias: since we have only an estimate of the bias, the covariance includes the corresponding uncertainty
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

        // Update preintegrated measurements covariance: as in [2] we consider a first order propagation that
        // can be seen as a prediction phase in an EKF framework. In this implementation, contrarily to [2] we
        // consider the uncertainty of the bias selection and we keep correlation between biases and preintegrated measurements
        /* ----------------------------------------------------------------------------------------------------------------------- */
        Matrix3 Z_3x3 = Matrix3::Zero();
        Matrix3 I_3x3 = Matrix3::Identity();
        const Vector3 theta_i = Rot3::Logmap(deltaRij); // parametrization of so(3)
        const Matrix3 Jr_theta_i = Rot3::rightJacobianExpMapSO3(theta_i);

        Rot3 Rot_j = deltaRij * Rincr;
        const Vector3 theta_j = Rot3::Logmap(Rot_j); // parametrization of so(3)
        const Matrix3 Jrinv_theta_j = Rot3::rightJacobianExpMapSO3inverse(theta_j);

        // Single Jacobians to propagate covariance
        Matrix3 H_pos_pos    = I_3x3;
        Matrix3 H_pos_vel    = I_3x3 * deltaT;
        Matrix3 H_pos_angles = Z_3x3;

        Matrix3 H_vel_pos    = Z_3x3;
        Matrix3 H_vel_vel    = I_3x3;
        Matrix3 H_vel_angles = - deltaRij.matrix() * skewSymmetric(correctedAcc) * Jr_theta_i * deltaT;
        // analytic expression corresponding to the following numerical derivative
        // Matrix H_vel_angles = numericalDerivative11<LieVector, LieVector>(boost::bind(&PreIntegrateIMUObservations_delta_vel, correctedOmega, correctedAcc, deltaT, _1, deltaVij), theta_i);
        Matrix3 H_vel_biasacc = - deltaRij.matrix() * deltaT;

        Matrix3 H_angles_pos   = Z_3x3;
        Matrix3 H_angles_vel    = Z_3x3;
        Matrix3 H_angles_angles = Jrinv_theta_j * Rincr.inverse().matrix() * Jr_theta_i;
        Matrix3 H_angles_biasomega =- Jrinv_theta_j * Jr_theta_incr * deltaT;
        // analytic expression corresponding to the following numerical derivative
        // Matrix H_angles_angles = numericalDerivative11<LieVector, LieVector>(boost::bind(&PreIntegrateIMUObservations_delta_angles, correctedOmega, deltaT, _1), thetaij);

        // overall Jacobian wrt preintegrated measurements (df/dx)
        Matrix F(15,15);
        F << H_pos_pos,    H_pos_vel,     H_pos_angles,          Z_3x3,                     Z_3x3,
                H_vel_pos,     H_vel_vel,     H_vel_angles,      H_vel_biasacc,              Z_3x3,
                H_angles_pos,  H_angles_vel,  H_angles_angles,   Z_3x3,                         H_angles_biasomega,
                Z_3x3,         Z_3x3,         Z_3x3,             I_3x3,                         Z_3x3,
                Z_3x3,         Z_3x3,         Z_3x3,             Z_3x3,                         I_3x3;


        // first order uncertainty propagation
        // Optimized matrix multiplication   (1/deltaT) * G * measurementCovariance * G.transpose()

        Matrix G_measCov_Gt = Matrix::Zero(15,15);
        // BLOCK DIAGONAL TERMS
        G_measCov_Gt.block(0,0,3,3) = deltaT * measurementCovariance.block(0,0,3,3);

        G_measCov_Gt.block(3,3,3,3) = (1/deltaT) * (H_vel_biasacc)  *
            (measurementCovariance.block(3,3,3,3)  +  measurementCovariance.block(15,15,3,3) ) *
            (H_vel_biasacc.transpose());

        G_measCov_Gt.block(6,6,3,3) = (1/deltaT) *  (H_angles_biasomega) *
            (measurementCovariance.block(6,6,3,3)  +  measurementCovariance.block(18,18,3,3) ) *
            (H_angles_biasomega.transpose());

        G_measCov_Gt.block(9,9,3,3) = deltaT * measurementCovariance.block(9,9,3,3);

        G_measCov_Gt.block(12,12,3,3) = deltaT * measurementCovariance.block(12,12,3,3);

        // NEW OFF BLOCK DIAGONAL TERMS
        Matrix3 block23 = H_vel_biasacc * measurementCovariance.block(18,15,3,3) *  H_angles_biasomega.transpose();
        G_measCov_Gt.block(3,6,3,3) = block23;
        G_measCov_Gt.block(6,3,3,3) = block23.transpose();

        PreintMeasCov = F * PreintMeasCov * F.transpose() + G_measCov_Gt;

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

    typedef CombinedImuFactor This;
    typedef NoiseModelFactor6<Pose3,LieVector,Pose3,LieVector,imuBias::ConstantBias,imuBias::ConstantBias> Base;

    CombinedPreintegratedMeasurements preintegratedMeasurements_;
    Vector3 gravity_;
    Vector3 omegaCoriolis_;
    boost::optional<Pose3> body_P_sensor_;        ///< The pose of the sensor in the body frame

    bool use2ndOrderCoriolis_; ///< Controls whether higher order terms are included when calculating the Coriolis Effect

  public:

    /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
    typedef typename boost::shared_ptr<CombinedImuFactor> shared_ptr;
#else
      typedef boost::shared_ptr<CombinedImuFactor> shared_ptr;
#endif

    /** Default constructor - only use for serialization */
    CombinedImuFactor() : preintegratedMeasurements_(imuBias::ConstantBias(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix::Zero(6,6)) {}

    /** Constructor */
    CombinedImuFactor(
      Key pose_i, ///< previous pose key
      Key vel_i, ///< previous velocity key
      Key pose_j, ///< current pose key
      Key vel_j, ///< current velocity key
      Key bias_i, ///< previous bias key
      Key bias_j, ///< current bias key
        const CombinedPreintegratedMeasurements& preintegratedMeasurements, ///< Preintegrated IMU measurements
        const Vector3& gravity, ///< gravity vector
        const Vector3& omegaCoriolis, ///< rotation rate of inertial frame
        boost::optional<const Pose3&> body_P_sensor = boost::none, ///< The Pose of the sensor frame in the body frame
        const bool use2ndOrderCoriolis = false ///< When true, the second-order term is used in the calculation of the Coriolis Effect
    ) :
      Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.PreintMeasCov), pose_i, vel_i, pose_j, vel_j, bias_i, bias_j),
      preintegratedMeasurements_(preintegratedMeasurements),
      gravity_(gravity),
      omegaCoriolis_(omegaCoriolis),
      body_P_sensor_(body_P_sensor),
      use2ndOrderCoriolis_(use2ndOrderCoriolis){
    }

    virtual ~CombinedImuFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "CombinedImuFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ","
          << keyFormatter(this->key3()) << ","
          << keyFormatter(this->key4()) << ","
          << keyFormatter(this->key5()) << ","
          << keyFormatter(this->key6()) << ")\n";
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
    const CombinedPreintegratedMeasurements& preintegratedMeasurements() const {
      return preintegratedMeasurements_; }

    const Vector3& gravity() const { return gravity_; }

    const Vector3& omegaCoriolis() const { return omegaCoriolis_; }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const Pose3& pose_i, const LieVector& vel_i, const Pose3& pose_j, const LieVector& vel_j,
        const imuBias::ConstantBias& bias_i, const imuBias::ConstantBias& bias_j,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none,
        boost::optional<Matrix&> H5 = boost::none,
        boost::optional<Matrix&> H6 = boost::none) const
    {

      const double& deltaTij = preintegratedMeasurements_.deltaTij;
      const Vector3 biasAccIncr = bias_i.accelerometer() - preintegratedMeasurements_.biasHat.accelerometer();
      const Vector3 biasOmegaIncr = bias_i.gyroscope() - preintegratedMeasurements_.biasHat.gyroscope();

      // we give some shorter name to rotations and translations
      const Rot3 Rot_i = pose_i.rotation();
      const Rot3 Rot_j = pose_j.rotation();
      const Vector3 pos_i = pose_i.translation().vector();
      const Vector3 pos_j = pose_j.translation().vector();

      // We compute factor's Jacobians, according to [3]
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

      /*
        (*H1) <<
            // dfP/dRi
            Rot_i.matrix() * skewSymmetric(preintegratedMeasurements_.deltaPij
                + preintegratedMeasurements_.delPdelBiasOmega * biasOmegaIncr + preintegratedMeasurements_.delPdelBiasAcc * biasAccIncr),
                // dfP/dPi
                - Rot_i.matrix() + 0.5 * skewSymmetric(omegaCoriolis_) * skewSymmetric(omegaCoriolis_) * Rot_i.matrix() * deltaTij*deltaTij,
                // dfV/dRi
                Rot_i.matrix() * skewSymmetric(preintegratedMeasurements_.deltaVij
                    + preintegratedMeasurements_.delVdelBiasOmega * biasOmegaIncr + preintegratedMeasurements_.delVdelBiasAcc * biasAccIncr),
                    // dfV/dPi
                    skewSymmetric(omegaCoriolis_) * skewSymmetric(omegaCoriolis_) * Rot_i.matrix() * deltaTij,
                    // dfR/dRi
                    Jrinv_fRhat *  (- Rot_j.between(Rot_i).matrix() - fRhat.inverse().matrix() * Jtheta),
                    // dfR/dPi
                    Matrix3::Zero(),
                    //dBiasAcc/dPi
                    Matrix3::Zero(), Matrix3::Zero(),
                    //dBiasOmega/dPi
                    Matrix3::Zero(), Matrix3::Zero();
          */
      if(H1) {
        H1->resize(15,6);

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
      Matrix3::Zero(),
      //dBiasAcc/dPi
      Matrix3::Zero(), Matrix3::Zero(),
      //dBiasOmega/dPi
      Matrix3::Zero(), Matrix3::Zero();
      }

      if(H2) {
        H2->resize(15,3);
        (*H2) <<
            // dfP/dVi
            - Matrix3::Identity() * deltaTij
            + skewSymmetric(omegaCoriolis_) * deltaTij * deltaTij,  // Coriolis term - we got rid of the 2 wrt ins paper
            // dfV/dVi
            - Matrix3::Identity()
      + 2 * skewSymmetric(omegaCoriolis_) * deltaTij, // Coriolis term
      // dfR/dVi
      Matrix3::Zero(),
      //dBiasAcc/dVi
      Matrix3::Zero(),
      //dBiasOmega/dVi
      Matrix3::Zero();
      }

      if(H3) {

        H3->resize(15,6);
        (*H3) <<
            // dfP/dPosej
            Matrix3::Zero(), Rot_j.matrix(),
            // dfV/dPosej
            Matrix::Zero(3,6),
            // dfR/dPosej
            Jrinv_fRhat *  ( Matrix3::Identity() ), Matrix3::Zero(),
            //dBiasAcc/dPosej
            Matrix3::Zero(), Matrix3::Zero(),
            //dBiasOmega/dPosej
            Matrix3::Zero(), Matrix3::Zero();
      }

      if(H4) {
        H4->resize(15,3);
        (*H4) <<
            // dfP/dVj
            Matrix3::Zero(),
            // dfV/dVj
            Matrix3::Identity(),
            // dfR/dVj
            Matrix3::Zero(),
            //dBiasAcc/dVj
            Matrix3::Zero(),
            //dBiasOmega/dVj
            Matrix3::Zero();
      }

      if(H5) {
        const Matrix3 Jrinv_theta_bc = Rot3::rightJacobianExpMapSO3inverse(theta_biascorrected);
        const Matrix3 Jr_JbiasOmegaIncr = Rot3::rightJacobianExpMapSO3(preintegratedMeasurements_.delRdelBiasOmega * biasOmegaIncr);
        const Matrix3 JbiasOmega = Jr_theta_bcc * Jrinv_theta_bc * Jr_JbiasOmegaIncr * preintegratedMeasurements_.delRdelBiasOmega;

        H5->resize(15,6);
        (*H5) <<
            // dfP/dBias_i
            - Rot_i.matrix() * preintegratedMeasurements_.delPdelBiasAcc,
            - Rot_i.matrix() * preintegratedMeasurements_.delPdelBiasOmega,
            // dfV/dBias_i
            - Rot_i.matrix() * preintegratedMeasurements_.delVdelBiasAcc,
            - Rot_i.matrix() * preintegratedMeasurements_.delVdelBiasOmega,
            // dfR/dBias_i
            Matrix::Zero(3,3),
            Jrinv_fRhat * ( - fRhat.inverse().matrix() * JbiasOmega),
            //dBiasAcc/dBias_i
            -Matrix3::Identity(), Matrix3::Zero(),
            //dBiasOmega/dBias_i
            Matrix3::Zero(), -Matrix3::Identity();
      }

      if(H6) {

          H6->resize(15,6);
          (*H6) <<
                  // dfP/dBias_j
                  Matrix3::Zero(), Matrix3::Zero(),
                  // dfV/dBias_j
                  Matrix3::Zero(), Matrix3::Zero(),
                  // dfR/dBias_j
                  Matrix3::Zero(), Matrix3::Zero(),
                  //dBiasAcc/dBias_j
                  Matrix3::Identity(), Matrix3::Zero(),
                  //dBiasOmega/dBias_j
                  Matrix3::Zero(), Matrix3::Identity();
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

      const Vector3 fbiasAcc = bias_j.accelerometer() - bias_i.accelerometer();

      const Vector3 fbiasOmega = bias_j.gyroscope() - bias_i.gyroscope();

      Vector r(15); r << fp, fv, fR, fbiasAcc, fbiasOmega; // vector of size 15

      return r;
    }


    /** predicted states from IMU */
    static void Predict(const Pose3& pose_i, const LieVector& vel_i, Pose3& pose_j, LieVector& vel_j,
        const imuBias::ConstantBias& bias_i, imuBias::ConstantBias& bias_j,
        const CombinedPreintegratedMeasurements& preintegratedMeasurements,
        const Vector3& gravity, const Vector3& omegaCoriolis, boost::optional<const Pose3&> body_P_sensor = boost::none,
        const bool use2ndOrderCoriolis = false)
    {

      const double& deltaTij = preintegratedMeasurements.deltaTij;
      const Vector3 biasAccIncr = bias_i.accelerometer() - preintegratedMeasurements.biasHat.accelerometer();
      const Vector3 biasOmegaIncr = bias_i.gyroscope() - preintegratedMeasurements.biasHat.gyroscope();

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

      bias_j = bias_i;
    }


  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor6",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(preintegratedMeasurements_);
      ar & BOOST_SERIALIZATION_NVP(gravity_);
      ar & BOOST_SERIALIZATION_NVP(omegaCoriolis_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
    }
  }; // \class CombinedImuFactor

  typedef CombinedImuFactor::CombinedPreintegratedMeasurements CombinedImuFactorPreintegratedMeasurements;

} /// namespace gtsam

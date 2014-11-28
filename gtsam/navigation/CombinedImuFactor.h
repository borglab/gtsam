/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  CombinedImuFactor.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/base/debug.h>

/* External or standard includes */
#include <ostream>


namespace gtsam {

  /**
   * Struct to hold all state variables of CombinedImuFactor
   * returned by predict function
   */
  struct PoseVelocityBias {
    Pose3 pose;
    Vector3 velocity;
    imuBias::ConstantBias bias;

    PoseVelocityBias(const Pose3& _pose, const Vector3& _velocity,
        const imuBias::ConstantBias _bias) :
        pose(_pose), velocity(_velocity), bias(_bias) {
    }
  };

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

  class CombinedImuFactor: public NoiseModelFactor6<Pose3,Vector3,Pose3,Vector3,imuBias::ConstantBias,imuBias::ConstantBias> {

    static const Matrix3 Z_3x3;
    static const Matrix3 I_3x3;

  public:

    /** Struct to store results of preintegrating IMU measurements.  Can be build
     * incrementally so as to avoid costly integration at time of factor construction. */

    /** CombinedPreintegratedMeasurements accumulates (integrates) the IMU measurements (rotation rates and accelerations)
     * and the corresponding covariance matrix. The measurements are then used to build the Preintegrated IMU factor*/
    class CombinedPreintegratedMeasurements {
      friend class CombinedImuFactor;
    protected:
      imuBias::ConstantBias biasHat_; ///< Acceleration and angular rate bias values used during preintegration
      Eigen::Matrix<double,21,21> measurementCovariance_; ///< (Raw measurements uncertainty) Covariance of the vector
      ///< [integrationError measuredAcc measuredOmega biasAccRandomWalk biasOmegaRandomWalk biasAccInit biasOmegaInit] in R^(21 x 21)

      Vector3 deltaPij_; ///< Preintegrated relative position (does not take into account velocity at time i, see deltap+, in [2]) (in frame i)
      Vector3 deltaVij_; ///< Preintegrated relative velocity (in global frame)
      Rot3 deltaRij_; ///< Preintegrated relative orientation (in frame i)
      double deltaTij_; ///< Time interval from i to j

      Matrix3 delPdelBiasAcc_; ///< Jacobian of preintegrated position w.r.t. acceleration bias
      Matrix3 delPdelBiasOmega_; ///< Jacobian of preintegrated position w.r.t. angular rate bias
      Matrix3 delVdelBiasAcc_; ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
      Matrix3 delVdelBiasOmega_; ///< Jacobian of preintegrated velocity w.r.t. angular rate bias
      Matrix3 delRdelBiasOmega_; ///< Jacobian of preintegrated rotation w.r.t. angular rate bias
      Eigen::Matrix<double,15,15> PreintMeasCov_; ///< Covariance matrix of the preintegrated measurements (first-order propagation from *measurementCovariance*)
      bool use2ndOrderIntegration_; ///< Controls the order of integration
//    public:
      ///< In the combined factor is also includes the biases and keeps the correlation between the preintegrated measurements and the biases
      ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION BiasAcc BiasOmega]
      /** Default constructor, initialize with no IMU measurements */
    public:
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
      ) : biasHat_(bias), deltaPij_(Vector3::Zero()), deltaVij_(Vector3::Zero()), deltaTij_(0.0),
      delPdelBiasAcc_(Z_3x3), delPdelBiasOmega_(Z_3x3),
      delVdelBiasAcc_(Z_3x3), delVdelBiasOmega_(Z_3x3),
      delRdelBiasOmega_(Z_3x3),
      use2ndOrderIntegration_(use2ndOrderIntegration)
      {
          // COVARIANCE OF: [Integration AccMeasurement OmegaMeasurement BiasAccRandomWalk BiasOmegaRandomWalk (BiasAccInit BiasOmegaInit)] SIZE (21x21)
        measurementCovariance_.setZero();
        measurementCovariance_.block<3,3>(0,0) = integrationErrorCovariance;
        measurementCovariance_.block<3,3>(3,3) = measuredAccCovariance;
        measurementCovariance_.block<3,3>(6,6) = measuredOmegaCovariance;
        measurementCovariance_.block<3,3>(9,9) = biasAccCovariance;
        measurementCovariance_.block<3,3>(12,12) = biasOmegaCovariance;
        measurementCovariance_.block<6,6>(15,15) = biasAccOmegaInit;

        PreintMeasCov_.setZero();
      }

      // TODO: in what context is this constructor used and why do you init to zero?
      // measurementCovariance_ was is not initialized, BTW
      CombinedPreintegratedMeasurements() :
      biasHat_(imuBias::ConstantBias()), deltaPij_(Vector3::Zero()), deltaVij_(Vector3::Zero()), deltaTij_(0.0),
      delPdelBiasAcc_(Z_3x3), delPdelBiasOmega_(Z_3x3),
      delVdelBiasAcc_(Z_3x3), delVdelBiasOmega_(Z_3x3),
      delRdelBiasOmega_(Z_3x3),
      use2ndOrderIntegration_(false) ///< Controls the order of integration
      {
        PreintMeasCov_.setZero();
      }

      /** print */
      void print(const std::string& s = "Preintegrated Measurements:") const {
        std::cout << s << std::endl;
        biasHat_.print("  biasHat");
        std::cout << "  deltaTij " << deltaTij_ << std::endl;
        std::cout << "  deltaPij [ " << deltaPij_.transpose() << " ]" << std::endl;
        std::cout << "  deltaVij [ " << deltaVij_.transpose() << " ]" << std::endl;
        deltaRij_.print("  deltaRij ");
        std::cout << "  measurementCovariance [ " << measurementCovariance_ << " ]" << std::endl;
        std::cout << "  PreintMeasCov [ " << PreintMeasCov_ << " ]" << std::endl;
      }

      /** equals */
      bool equals(const CombinedPreintegratedMeasurements& expected, double tol=1e-9) const {
        return biasHat_.equals(expected.biasHat_, tol)
            && equal_with_abs_tol(measurementCovariance_, expected.measurementCovariance_, tol)
            && equal_with_abs_tol(deltaPij_, expected.deltaPij_, tol)
            && equal_with_abs_tol(deltaVij_, expected.deltaVij_, tol)
            && deltaRij_.equals(expected.deltaRij_, tol)
            && std::fabs(deltaTij_ - expected.deltaTij_) < tol
            && equal_with_abs_tol(delPdelBiasAcc_, expected.delPdelBiasAcc_, tol)
            && equal_with_abs_tol(delPdelBiasOmega_, expected.delPdelBiasOmega_, tol)
            && equal_with_abs_tol(delVdelBiasAcc_, expected.delVdelBiasAcc_, tol)
            && equal_with_abs_tol(delVdelBiasOmega_, expected.delVdelBiasOmega_, tol)
            && equal_with_abs_tol(delRdelBiasOmega_, expected.delRdelBiasOmega_, tol);
      }

      void resetIntegration(){
        deltaPij_ = Vector3::Zero();
        deltaVij_ = Vector3::Zero();
        deltaRij_ = Rot3();
        deltaTij_ = 0.0;
        delPdelBiasAcc_ = Z_3x3;
        delPdelBiasOmega_ = Z_3x3;
        delVdelBiasAcc_ = Z_3x3;
        delVdelBiasOmega_ = Z_3x3;
        delRdelBiasOmega_ = Z_3x3;
        PreintMeasCov_.setZero();
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
        Vector3 correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
        Vector3 correctedOmega = biasHat_.correctGyroscope(measuredOmega);

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
          delPdelBiasAcc_ += delVdelBiasAcc_ * deltaT;
          delPdelBiasOmega_ += delVdelBiasOmega_ * deltaT;
        }else{
          delPdelBiasAcc_ += delVdelBiasAcc_ * deltaT - 0.5 * deltaRij_.matrix() * deltaT*deltaT;
          delPdelBiasOmega_ += delVdelBiasOmega_ * deltaT - 0.5 * deltaRij_.matrix()
                                        * skewSymmetric(biasHat_.correctAccelerometer(measuredAcc)) * deltaT*deltaT * delRdelBiasOmega_;
        }

        delVdelBiasAcc_ += -deltaRij_.matrix() * deltaT;
        delVdelBiasOmega_ += -deltaRij_.matrix() * skewSymmetric(correctedAcc) * deltaT * delRdelBiasOmega_;
        delRdelBiasOmega_ = Rincr.inverse().matrix() * delRdelBiasOmega_ - Jr_theta_incr  * deltaT;

        // Update preintegrated measurements covariance: as in [2] we consider a first order propagation that
        // can be seen as a prediction phase in an EKF framework. In this implementation, contrarily to [2] we
        // consider the uncertainty of the bias selection and we keep correlation between biases and preintegrated measurements
        /* ----------------------------------------------------------------------------------------------------------------------- */
        const Vector3 theta_i = Rot3::Logmap(deltaRij_); // parametrization of so(3)
        const Matrix3 Jr_theta_i = Rot3::rightJacobianExpMapSO3(theta_i);

        Rot3 Rot_j = deltaRij_ * Rincr;
        const Vector3 theta_j = Rot3::Logmap(Rot_j); // parametrization of so(3)
        const Matrix3 Jrinv_theta_j = Rot3::rightJacobianExpMapSO3inverse(theta_j);

        // Single Jacobians to propagate covariance
        Matrix3 H_pos_pos    = I_3x3;
        Matrix3 H_pos_vel    = I_3x3 * deltaT;
        Matrix3 H_pos_angles = Z_3x3;

        Matrix3 H_vel_pos    = Z_3x3;
        Matrix3 H_vel_vel    = I_3x3;
        Matrix3 H_vel_angles = - deltaRij_.matrix() * skewSymmetric(correctedAcc) * Jr_theta_i * deltaT;
        // analytic expression corresponding to the following numerical derivative
        // Matrix H_vel_angles = numericalDerivative11<LieVector, LieVector>(boost::bind(&PreIntegrateIMUObservations_delta_vel, correctedOmega, correctedAcc, deltaT, _1, deltaVij), theta_i);
        Matrix3 H_vel_biasacc = - deltaRij_.matrix() * deltaT;

        Matrix3 H_angles_pos   = Z_3x3;
        Matrix3 H_angles_vel    = Z_3x3;
        Matrix3 H_angles_angles = Jrinv_theta_j * Rincr.inverse().matrix() * Jr_theta_i;
        Matrix3 H_angles_biasomega =- Jrinv_theta_j * Jr_theta_incr * deltaT;
        // analytic expression corresponding to the following numerical derivative
        // Matrix H_angles_angles = numericalDerivative11<Vector3, Vector3>(boost::bind(&PreIntegrateIMUObservations_delta_angles, correctedOmega, deltaT, _1), thetaij);

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
        G_measCov_Gt.block<3,3>(0,0) = deltaT * measurementCovariance_.block<3,3>(0,0);

        G_measCov_Gt.block<3,3>(3,3) = (1/deltaT) * (H_vel_biasacc)  *
            (measurementCovariance_.block<3,3>(3,3)  +  measurementCovariance_.block<3,3>(15,15) ) *
            (H_vel_biasacc.transpose());

        G_measCov_Gt.block<3,3>(6,6) = (1/deltaT) *  (H_angles_biasomega) *
            (measurementCovariance_.block<3,3>(6,6)  +  measurementCovariance_.block<3,3>(18,18) ) *
            (H_angles_biasomega.transpose());

        G_measCov_Gt.block<3,3>(9,9) = deltaT * measurementCovariance_.block<3,3>(9,9);

        G_measCov_Gt.block<3,3>(12,12) = deltaT * measurementCovariance_.block<3,3>(12,12);

        // NEW OFF BLOCK DIAGONAL TERMS
        Matrix3 block23 = H_vel_biasacc * measurementCovariance_.block<3,3>(18,15) *  H_angles_biasomega.transpose();
        G_measCov_Gt.block<3,3>(3,6) = block23;
        G_measCov_Gt.block<3,3>(6,3) = block23.transpose();

        PreintMeasCov_ = F * PreintMeasCov_ * F.transpose() + G_measCov_Gt;

        // Update preintegrated measurements
        /* ----------------------------------------------------------------------------------------------------------------------- */
        if(!use2ndOrderIntegration_){
          deltaPij_ += deltaVij_ * deltaT;
        }else{
          deltaPij_ += deltaVij_ * deltaT + 0.5 * deltaRij_.matrix() * biasHat_.correctAccelerometer(measuredAcc) * deltaT*deltaT;
        }
        deltaVij_ += deltaRij_.matrix() * correctedAcc * deltaT;
        deltaRij_ = deltaRij_ * Rincr;
        deltaTij_ += deltaT;
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
      Matrix measurementCovariance() const {return measurementCovariance_;}
      Matrix deltaRij() const {return deltaRij_.matrix();}
      double deltaTij() const{return deltaTij_;}
      Vector deltaPij() const {return deltaPij_;}
      Vector deltaVij() const {return deltaVij_;}
      Vector biasHat() const { return biasHat_.vector();}
      Matrix delPdelBiasAcc() const { return delPdelBiasAcc_;}
      Matrix delPdelBiasOmega() const { return delPdelBiasOmega_;}
      Matrix delVdelBiasAcc() const { return delVdelBiasAcc_;}
      Matrix delVdelBiasOmega() const { return delVdelBiasOmega_;}
      Matrix delRdelBiasOmega() const{ return delRdelBiasOmega_;}
      Matrix PreintMeasCov() const { return PreintMeasCov_;}

    private:
      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(biasHat_);
        ar & BOOST_SERIALIZATION_NVP(measurementCovariance_);
        ar & BOOST_SERIALIZATION_NVP(deltaPij_);
        ar & BOOST_SERIALIZATION_NVP(deltaVij_);
        ar & BOOST_SERIALIZATION_NVP(deltaRij_);
        ar & BOOST_SERIALIZATION_NVP(deltaTij_);
        ar & BOOST_SERIALIZATION_NVP(delPdelBiasAcc_);
        ar & BOOST_SERIALIZATION_NVP(delPdelBiasOmega_);
        ar & BOOST_SERIALIZATION_NVP(delVdelBiasAcc_);
        ar & BOOST_SERIALIZATION_NVP(delVdelBiasOmega_);
        ar & BOOST_SERIALIZATION_NVP(delRdelBiasOmega_);
      }
    };

  private:

    typedef CombinedImuFactor This;
    typedef NoiseModelFactor6<Pose3,Vector3,Pose3,Vector3,imuBias::ConstantBias,imuBias::ConstantBias> Base;

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
    CombinedImuFactor() : preintegratedMeasurements_(imuBias::ConstantBias(), Z_3x3, Z_3x3, Z_3x3, Z_3x3, Z_3x3, Matrix::Zero(6,6)) {}

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
      Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.PreintMeasCov_), pose_i, vel_i, pose_j, vel_j, bias_i, bias_j),
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
    Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
        const imuBias::ConstantBias& bias_i, const imuBias::ConstantBias& bias_j,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none,
        boost::optional<Matrix&> H5 = boost::none,
        boost::optional<Matrix&> H6 = boost::none) const
    {

      const double& deltaTij = preintegratedMeasurements_.deltaTij_;
      const Vector3 biasAccIncr = bias_i.accelerometer() - preintegratedMeasurements_.biasHat_.accelerometer();
      const Vector3 biasOmegaIncr = bias_i.gyroscope() - preintegratedMeasurements_.biasHat_.gyroscope();

      // we give some shorter name to rotations and translations
      const Rot3 Rot_i = pose_i.rotation();
      const Rot3 Rot_j = pose_j.rotation();
      const Vector3 pos_i = pose_i.translation().vector();
      const Vector3 pos_j = pose_j.translation().vector();

      // We compute factor's Jacobians, according to [3]
      /* ---------------------------------------------------------------------------------------------------- */
      const Rot3 deltaRij_biascorrected = preintegratedMeasurements_.deltaRij_.retract(preintegratedMeasurements_.delRdelBiasOmega_ * biasOmegaIncr, Rot3::EXPMAP);
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

      /* TODO why is this commented out. Put it on a branch but remove from develop?
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
                    Z_3x3,
                    //dBiasAcc/dPi
                    Z_3x3, Z_3x3,
                    //dBiasOmega/dPi
                    Z_3x3, Z_3x3;
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
          dfVdPi = Z_3x3;
        }

    (*H1) <<
      // dfP/dRi
      Rot_i.matrix() * skewSymmetric(preintegratedMeasurements_.deltaPij_
        + preintegratedMeasurements_.delPdelBiasOmega_ * biasOmegaIncr + preintegratedMeasurements_.delPdelBiasAcc_ * biasAccIncr),
      // dfP/dPi
      dfPdPi,
      // dfV/dRi
      Rot_i.matrix() * skewSymmetric(preintegratedMeasurements_.deltaVij_
        + preintegratedMeasurements_.delVdelBiasOmega_ * biasOmegaIncr + preintegratedMeasurements_.delVdelBiasAcc_ * biasAccIncr),
      // dfV/dPi
      dfVdPi,
      // dfR/dRi
      Jrinv_fRhat *  (- Rot_j.between(Rot_i).matrix() - fRhat.inverse().matrix() * Jtheta),
      // dfR/dPi
      Z_3x3,
      //dBiasAcc/dPi
      Z_3x3, Z_3x3,
      //dBiasOmega/dPi
      Z_3x3, Z_3x3;
      }

      if(H2) {
        H2->resize(15,3);
        (*H2) <<
            // dfP/dVi
            - I_3x3 * deltaTij
            + skewSymmetric(omegaCoriolis_) * deltaTij * deltaTij,  // Coriolis term - we got rid of the 2 wrt ins paper
            // dfV/dVi
            - I_3x3
      + 2 * skewSymmetric(omegaCoriolis_) * deltaTij, // Coriolis term
      // dfR/dVi
      Z_3x3,
      //dBiasAcc/dVi
      Z_3x3,
      //dBiasOmega/dVi
      Z_3x3;
      }

      if(H3) {

        H3->resize(15,6);
        (*H3) <<
            // dfP/dPosej
            Z_3x3, Rot_j.matrix(),
            // dfV/dPosej
            Matrix::Zero(3,6),
            // dfR/dPosej
            Jrinv_fRhat *  ( I_3x3 ), Z_3x3,
            //dBiasAcc/dPosej
            Z_3x3, Z_3x3,
            //dBiasOmega/dPosej
            Z_3x3, Z_3x3;
      }

      if(H4) {
        H4->resize(15,3);
        (*H4) <<
            // dfP/dVj
            Z_3x3,
            // dfV/dVj
            I_3x3,
            // dfR/dVj
            Z_3x3,
            //dBiasAcc/dVj
            Z_3x3,
            //dBiasOmega/dVj
            Z_3x3;
      }

      if(H5) {
        const Matrix3 Jrinv_theta_bc = Rot3::rightJacobianExpMapSO3inverse(theta_biascorrected);
        const Matrix3 Jr_JbiasOmegaIncr = Rot3::rightJacobianExpMapSO3(preintegratedMeasurements_.delRdelBiasOmega_ * biasOmegaIncr);
        const Matrix3 JbiasOmega = Jr_theta_bcc * Jrinv_theta_bc * Jr_JbiasOmegaIncr * preintegratedMeasurements_.delRdelBiasOmega_;

        H5->resize(15,6);
        (*H5) <<
            // dfP/dBias_i
            - Rot_i.matrix() * preintegratedMeasurements_.delPdelBiasAcc_,
            - Rot_i.matrix() * preintegratedMeasurements_.delPdelBiasOmega_,
            // dfV/dBias_i
            - Rot_i.matrix() * preintegratedMeasurements_.delVdelBiasAcc_,
            - Rot_i.matrix() * preintegratedMeasurements_.delVdelBiasOmega_,
            // dfR/dBias_i
            Matrix::Zero(3,3),
            Jrinv_fRhat * ( - fRhat.inverse().matrix() * JbiasOmega),
            //dBiasAcc/dBias_i
            -I_3x3, Z_3x3,
            //dBiasOmega/dBias_i
            Z_3x3, -I_3x3;
      }

      if(H6) {

          H6->resize(15,6);
          (*H6) <<
                  // dfP/dBias_j
                  Z_3x3, Z_3x3,
                  // dfV/dBias_j
                  Z_3x3, Z_3x3,
                  // dfR/dBias_j
                  Z_3x3, Z_3x3,
                  //dBiasAcc/dBias_j
                  I_3x3, Z_3x3,
                  //dBiasOmega/dBias_j
                  Z_3x3, I_3x3;
      }

      // Evaluate residual error, according to [3]
      /* ---------------------------------------------------------------------------------------------------- */
      const Vector3 fp =
          pos_j - pos_i
          - Rot_i.matrix() * (preintegratedMeasurements_.deltaPij_
              + preintegratedMeasurements_.delPdelBiasAcc_ * biasAccIncr
              + preintegratedMeasurements_.delPdelBiasOmega_ * biasOmegaIncr)
              - vel_i * deltaTij
              + skewSymmetric(omegaCoriolis_) * vel_i * deltaTij*deltaTij  // Coriolis term - we got rid of the 2 wrt ins paper
              - 0.5 * gravity_ * deltaTij*deltaTij;

      const Vector3 fv =
          vel_j - vel_i - Rot_i.matrix() * (preintegratedMeasurements_.deltaVij_
              + preintegratedMeasurements_.delVdelBiasAcc_ * biasAccIncr
              + preintegratedMeasurements_.delVdelBiasOmega_ * biasOmegaIncr)
              + 2 * skewSymmetric(omegaCoriolis_) * vel_i * deltaTij  // Coriolis term
              - gravity_ * deltaTij;

      const Vector3 fR = Rot3::Logmap(fRhat);

      const Vector3 fbiasAcc = bias_j.accelerometer() - bias_i.accelerometer();

      const Vector3 fbiasOmega = bias_j.gyroscope() - bias_i.gyroscope();

      Vector r(15); r << fp, fv, fR, fbiasAcc, fbiasOmega; // vector of size 15

      return r;
    }


    /** predicted states from IMU */
    static PoseVelocityBias Predict(const Pose3& pose_i, const Vector3& vel_i,
        const imuBias::ConstantBias& bias_i,
        const CombinedPreintegratedMeasurements& preintegratedMeasurements,
        const Vector3& gravity, const Vector3& omegaCoriolis, boost::optional<const Pose3&> body_P_sensor = boost::none,
        const bool use2ndOrderCoriolis = false)
    {

      const double& deltaTij = preintegratedMeasurements.deltaTij_;
      const Vector3 biasAccIncr = bias_i.accelerometer() - preintegratedMeasurements.biasHat_.accelerometer();
      const Vector3 biasOmegaIncr = bias_i.gyroscope() - preintegratedMeasurements.biasHat_.gyroscope();

      const Rot3 Rot_i = pose_i.rotation();
      const Vector3 pos_i = pose_i.translation().vector();

      // Predict state at time j
      /* ---------------------------------------------------------------------------------------------------- */
      Vector3 pos_j =  pos_i + Rot_i.matrix() * (preintegratedMeasurements.deltaPij_
          + preintegratedMeasurements.delPdelBiasAcc_ * biasAccIncr
          + preintegratedMeasurements.delPdelBiasOmega_ * biasOmegaIncr)
          + vel_i * deltaTij
          - skewSymmetric(omegaCoriolis) * vel_i * deltaTij*deltaTij  // Coriolis term - we got rid of the 2 wrt ins paper
          + 0.5 * gravity * deltaTij*deltaTij;

      Vector3 vel_j = Vector3(vel_i + Rot_i.matrix() * (preintegratedMeasurements.deltaVij_
          + preintegratedMeasurements.delVdelBiasAcc_ * biasAccIncr
          + preintegratedMeasurements.delVdelBiasOmega_ * biasOmegaIncr)
          - 2 * skewSymmetric(omegaCoriolis) * vel_i * deltaTij  // Coriolis term
          + gravity * deltaTij);

      if(use2ndOrderCoriolis){
        pos_j += - 0.5 * skewSymmetric(omegaCoriolis) * skewSymmetric(omegaCoriolis) * pos_i * deltaTij*deltaTij;  // 2nd order coriolis term for position
        vel_j += - skewSymmetric(omegaCoriolis) * skewSymmetric(omegaCoriolis) * pos_i * deltaTij; // 2nd order term for velocity
      }

      const Rot3 deltaRij_biascorrected = preintegratedMeasurements.deltaRij_.retract(preintegratedMeasurements.delRdelBiasOmega_ * biasOmegaIncr, Rot3::EXPMAP);
      // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)
      Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);
      Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected  -
          Rot_i.inverse().matrix() * omegaCoriolis * deltaTij; // Coriolis term
      const Rot3 deltaRij_biascorrected_corioliscorrected =
          Rot3::Expmap( theta_biascorrected_corioliscorrected );
      const Rot3 Rot_j = Rot_i.compose( deltaRij_biascorrected_corioliscorrected  );

      Pose3 pose_j = Pose3( Rot_j, Point3(pos_j) );

      return PoseVelocityBias(pose_j, vel_j, bias_i);
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

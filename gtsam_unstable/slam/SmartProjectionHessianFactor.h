/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionFactor.h
 * @brief Basic bearing factor from 2D measurement
 * @author Chris Beall
 * @author Luca Carlone
 * @author Zsolt Kira
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <vector>
#include <gtsam_unstable/geometry/triangulation.h>
#include <boost/optional.hpp>
//#include <boost/assign.hpp>

static bool isDebug=false;

namespace gtsam {

  // default threshold for selective relinearization
  static double defaultLinThreshold = -1; // 1e-7; // 0.01
  // default threshold for retriangulation
  static double defaultTriangThreshold = 1e-5;
  // default threshold for rank deficient triangulation
  static double defaultRankTolerance = 1; // this value may be scenario-dependent and has to be larger in  presence of larger noise
  // if set to true will use the rotation-only version for degenerate cases
  static bool manageDegeneracy = true;

  /**
   * Structure for storing some state memory, used to speed up optimization
   * @addtogroup SLAM
   */
  class SmartProjectionHessianFactorState {
  public:

    static int lastID;
    int ID;

    SmartProjectionHessianFactorState() {
      ID = lastID++;
      calculatedHessian = false;
    }

    // Linearization point
    Values values;
    std::vector<Pose3> cameraPosesLinearization;

    // Triangulation at current linearization point
    Point3 point;
    std::vector<Pose3> cameraPosesTriangulation;
    bool degenerate;
    bool cheiralityException;

    // Overall reprojection error
    double overallError;
    std::vector<Pose3> cameraPosesError;

    // Hessian representation (after Schur complement)
    bool calculatedHessian;
    Matrix H;
    Vector gs_vector;
    std::vector<Matrix> Gs;
    std::vector<Vector> gs;
    double f;

    // C = Hl'Hl
    // Cinv = inv(Hl'Hl)
    // Matrix3 Cinv;
    // E = Hx'Hl
    // w = Hl'b
  };

  int SmartProjectionHessianFactorState::lastID = 0;

  /**
   * The calibration is known here.
   * @addtogroup SLAM
   */
  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class SmartProjectionHessianFactor: public NonlinearFactor {
  protected:

    // Keep a copy of measurement and calibration for I/O
    std::vector<Point2> measured_;                    ///< 2D measurement for each of the m views
    std::vector< SharedNoiseModel > noise_;   ///< noise model used
    ///< (important that the order is the same as the keys that we use to create the factor)
    std::vector< boost::shared_ptr<CALIBRATION> > K_all_;  ///< shared pointer to calibration object (one for each camera)

    double retriangulationThreshold; ///< threshold to decide whether to re-triangulate

    double rankTolerance; ///< threshold to decide whether triangulation is degenerate

    double linearizationThreshold; ///< threshold to decide whether to re-linearize

    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame (one for each camera)

    boost::shared_ptr<SmartProjectionHessianFactorState> state_;

    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  public:

    /// shorthand for base class type
    typedef NonlinearFactor Base;

    /// shorthand for this class
    typedef SmartProjectionHessianFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// shorthand for smart projection factor state variable
    typedef boost::shared_ptr<SmartProjectionHessianFactorState> SmartFactorStatePtr;

    /**
     * Constructor
     * @param poseKeys is the set of indices corresponding to the cameras observing the same landmark
     * @param measured is the 2m dimensional location of the projection of a single landmark in the m views (the measurements)
     * @param model is the standard deviation (current version assumes that the uncertainty is the same for all views)
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    SmartProjectionHessianFactor(
        const double rankTol = defaultRankTolerance,
        const double linThreshold = defaultLinThreshold,
        boost::optional<POSE> body_P_sensor = boost::none,
        SmartFactorStatePtr state = SmartFactorStatePtr(new SmartProjectionHessianFactorState())) :
          retriangulationThreshold(defaultTriangThreshold), rankTolerance(rankTol),
          linearizationThreshold(linThreshold), body_P_sensor_(body_P_sensor),
          state_(state), throwCheirality_(false), verboseCheirality_(false) {}


    /** Virtual destructor */
    virtual ~SmartProjectionHessianFactor() {}

    /**
     * add a new measurement and pose key
     * @param measured is the 2m dimensional location of the projection of a single landmark in the m view (the measurement)
     * @param poseKey is the index corresponding to the camera observing the same landmark
     */
    void add(const Point2 measured_i, const Key poseKey_i, const SharedNoiseModel noise_i,
        const boost::shared_ptr<CALIBRATION> K_i) {
      measured_.push_back(measured_i);
      keys_.push_back(poseKey_i);
      noise_.push_back(noise_i);
      K_all_.push_back(K_i);
    }

    void add(std::vector< Point2 > measurements, std::vector< Key > poseKeys, std::vector< SharedNoiseModel > noises,
        std::vector< boost::shared_ptr<CALIBRATION> > Ks) {
      for(size_t i = 0; i < measurements.size(); i++) {
        measured_.push_back(measurements.at(i));
        keys_.push_back(poseKeys.at(i));
        noise_.push_back(noises.at(i));
        K_all_.push_back(Ks.at(i));
      }
    }

    void add(std::vector< Point2 > measurements, std::vector< Key > poseKeys, const SharedNoiseModel noise,
        const boost::shared_ptr<CALIBRATION> K) {
      for(size_t i = 0; i < measurements.size(); i++) {
        measured_.push_back(measurements.at(i));
        keys_.push_back(poseKeys.at(i));
        noise_.push_back(noise);
        K_all_.push_back(K);
      }
    }

    // This function checks if the new linearization point is the same as the one used for previous triangulation
    // (if not, a new triangulation is needed)
    static bool decideIfTriangulate(std::vector<Pose3> cameraPoses, std::vector<Pose3> oldPoses, double retriangulationThreshold) {
      // several calls to linearize will be done from the same linearization point, hence it is not needed to re-triangulate
      // Note that this is not yet "selecting linearization", that will come later, and we only check if the
      // current linearization is the "same" (up to tolerance) w.r.t. the last time we triangulated the point

      // if we do not have a previous linearization point or the new linearization point includes more poses
      if(oldPoses.empty() || (cameraPoses.size() != oldPoses.size()))
        return true;

      for(size_t i = 0; i < cameraPoses.size(); i++) {
          if (!cameraPoses[i].equals(oldPoses[i], retriangulationThreshold)) {
            return true; // at least two poses are different, hence we retriangulate
          }
      }
      return false; // if we arrive to this point all poses are the same and we don't need re-triangulation
    }

    // This function checks if the new linearization point is 'close'  to the previous one used for linearization
    // (if not, a new linearization is needed)
    static bool decideIfLinearize(std::vector<Pose3> cameraPoses, std::vector<Pose3> oldPoses, double linearizationThreshold) {
      // "selective linearization"
      // The function evaluates how close are the old and the new poses, transformed in the ref frame of the first pose
      // (we only care about the "rigidity" of the poses, not about their absolute pose)

      // if we do not have a previous linearization point or the new linearization point includes more poses
      if(oldPoses.empty() || (cameraPoses.size() != oldPoses.size()))
        return true;

      Pose3 firstCameraPose;
      Pose3 firstCameraPoseOld;

      for(size_t i = 0; i < cameraPoses.size(); i++) {

          if(i==0){ // we store the initial pose, this is useful for selective re-linearization
            firstCameraPose = cameraPoses[i];
            firstCameraPoseOld = oldPoses[i];
            continue;
          }

          // we compare the poses in the frame of the first pose
          Pose3 localCameraPose = firstCameraPose.between(cameraPoses[i]);
          Pose3 localCameraPoseOld = firstCameraPoseOld.between(oldPoses[i]);

          if (!localCameraPose.equals(localCameraPoseOld, linearizationThreshold)) {
            return true; // at least two "relative" poses are different, hence we re-linerize
          }
      }
      return false; // if we arrive to this point all poses are the same and we don't need re-linerize
    }


    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "SmartProjectionHessianFactor, z = \n ";
      BOOST_FOREACH(const Point2& p, measured_) {
        std::cout << "measurement, p = "<< p << std::endl;
      }
      BOOST_FOREACH(const SharedNoiseModel& noise_i, noise_) {
        noise_i->print("noise model = ");
      }
      BOOST_FOREACH(const boost::shared_ptr<CALIBRATION>& K, K_all_) {
        K->print("calibration = ");
      }

      if(this->body_P_sensor_){
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      }
      Base::print("", keyFormatter);
    }

    /// equals
    virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This*>(&p);

      bool areMeasurementsEqual = true;
      for(size_t i = 0; i < measured_.size(); i++) {
        if(this->measured_.at(i).equals(e->measured_.at(i), tol) == false)
          areMeasurementsEqual = false;
        break;
      }

      return e
          && Base::equals(p, tol)
          && areMeasurementsEqual
          //&& this->K_->equals(*e->K_all_, tol);
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /// get the dimension of the factor (number of rows on linearization)
    virtual size_t dim() const {
        return 6*keys_.size();
    }

    /// linearize returns a Hessianfactor that is an approximation of error(p)
    virtual boost::shared_ptr<GaussianFactor> linearize(const Values& values) const {

      bool blockwise = false;                  // the full matrix version in faster
      int dim_landmark = 3;                    // for degenerate instances this will become 2 (direction-only information)

      // Create structures for Hessian Factors
      unsigned int numKeys = keys_.size();
      if(isDebug) {std::cout<< " numKeys = "<< numKeys<<std::endl; }
      std::vector<Index> js;
      std::vector<Matrix> Gs(numKeys*(numKeys+1)/2);
      std::vector<Vector> gs(numKeys);
      double f=0;

      // Collect all poses (Cameras)
      std::vector<Pose3> cameraPoses;
      BOOST_FOREACH(const Key& k, keys_) {
        Pose3 cameraPose;
        if(body_P_sensor_)    {   cameraPose = values.at<Pose3>(k).compose(*body_P_sensor_);}
        else                  { cameraPose = values.at<Pose3>(k);}
        cameraPoses.push_back(cameraPose);
      }

      if(cameraPoses.size() < 2){ // if we have a single pose the corresponding factor is uninformative
        state_->degenerate = true;
        BOOST_FOREACH(gtsam::Matrix& m, Gs) m = zeros(6, 6);
        BOOST_FOREACH(Vector& v, gs) v = zero(6);
        return HessianFactor::shared_ptr(new HessianFactor(keys_, Gs, gs, f)); // TODO: Debug condition, uncomment when fixed
      }

      bool retriangulate = decideIfTriangulate(cameraPoses, state_->cameraPosesTriangulation, retriangulationThreshold);

      if(retriangulate) {// we store the current poses used for triangulation
        state_->cameraPosesTriangulation = cameraPoses;
      }

      if (retriangulate) {
        // We triangulate the 3D position of the landmark
        try {
          // std::cout << "triangulatePoint3 i \n" << rankTolerance << std::endl;
          state_->point = triangulatePoint3(cameraPoses, measured_, K_all_, rankTolerance);
          state_->degenerate = false;
          state_->cheiralityException = false;
        } catch( TriangulationUnderconstrainedException& e) {
          // if TriangulationUnderconstrainedException can be
          // 1) There is a single pose for triangulation - this should not happen because we checked the number of poses before
          // 2) The rank of the matrix used for triangulation is < 3: rotation-only, parallel cameras (or motion towards the landmark)
          // in the second case we want to use a rotation-only smart factor
          //std::cout << "Triangulation failed " << e.what() << std::endl; // point triangulated at infinity
          state_->degenerate = true;
          state_->cheiralityException = false;
        } catch( TriangulationCheiralityException& e) {
          // point is behind one of the cameras: can be the case of close-to-parallel cameras or may depend on outliers
          // we manage this case by either discarding the smart factor, or imposing a rotation-only constraint
          //std::cout << e.what() << std::end;
          state_->cheiralityException = true;
        }
      }

      if (!manageDegeneracy && (state_->cheiralityException || state_->degenerate) ){
        // std::cout << "In linearize: exception" << std::endl;
        BOOST_FOREACH(gtsam::Matrix& m, Gs) m = zeros(6, 6);
        BOOST_FOREACH(Vector& v, gs) v = zero(6);
        return HessianFactor::shared_ptr(new HessianFactor(keys_, Gs, gs, f));
      }

      if (state_->cheiralityException || state_->degenerate){ // if we want to manage the exceptions with rotation-only factors
        state_->degenerate = true;
        dim_landmark = 2;
      }

      bool doLinearize;
      if (linearizationThreshold >= 0){//by convention if linearizationThreshold is negative we always relinearize
        // std::cout << "Temporary disabled" << std::endl;
        doLinearize = decideIfLinearize(cameraPoses, state_->cameraPosesLinearization, linearizationThreshold);
      }
      else{
        doLinearize = true;
      }

      if (doLinearize) {
        state_->cameraPosesLinearization = cameraPoses;
      }

      if(!doLinearize){ // return the previous Hessian factor
        // std::cout << "Using stored factors :) " << std::endl;
        return HessianFactor::shared_ptr(new HessianFactor(keys_, state_->Gs, state_->gs, state_->f));
      }

      if (blockwise == false){ // version with full matrix multiplication
        // ==========================================================================================================
        Matrix Hx2 = zeros(2 * numKeys, 6 * numKeys);
        Matrix Hl2 = zeros(2 * numKeys, dim_landmark);
        Vector b2 = zero(2 * numKeys);

        if(state_->degenerate){
          for(size_t i = 0; i < measured_.size(); i++) {
            Pose3 pose = cameraPoses.at(i);
            PinholeCamera<CALIBRATION> camera(pose, *K_all_.at(i));
            if(i==0){ // first pose
              state_->point = camera.backprojectPointAtInfinity(measured_.at(i));
              // 3D parametrization of point at infinity: [px py 1]
              // std::cout << "point_ " << state_->point<< std::endl;
            }
            Matrix Hxi, Hli;
            Vector bi = -( camera.projectPointAtInfinity(state_->point,Hxi,Hli) - measured_.at(i) ).vector();
            // std::cout << "Hxi \n" << Hxi<< std::endl;
            // std::cout << "Hli \n" << Hli<< std::endl;

            noise_.at(i)-> WhitenSystem(Hxi, Hli, bi);
            f += bi.squaredNorm();

            Hx2.block( 2*i, 6*i, 2, 6 ) = Hxi;
            Hl2.block( 2*i, 0, 2, 2  ) = Hli;

            subInsert(b2,bi,2*i);
          }
          // std::cout << "Hx2 \n" << Hx2<< std::endl;
          // std::cout << "Hl2 \n" << Hl2<< std::endl;
        }
        else{

          for(size_t i = 0; i < measured_.size(); i++) {
            Pose3 pose = cameraPoses.at(i);
            PinholeCamera<CALIBRATION> camera(pose, *K_all_.at(i));
            Matrix Hxi, Hli;

            Vector bi;
            try {
              bi = -( camera.project(state_->point,Hxi,Hli) - measured_.at(i) ).vector();
            } catch ( CheiralityException& e) {
              std::cout << "Cheirality exception " << state_->ID << std::endl;
              exit(EXIT_FAILURE);
            }
            noise_.at(i)-> WhitenSystem(Hxi, Hli, bi);
            f += bi.squaredNorm();

            Hx2.block( 2*i, 6*i, 2, 6 ) = Hxi;
            Hl2.block( 2*i, 0, 2, 3  ) = Hli;

            subInsert(b2,bi,2*i);
          }

        }

        // Shur complement trick
        Matrix H(6 * numKeys, 6 * numKeys);
        Matrix C2;
        Vector gs_vector;

        C2.noalias() = (Hl2.transpose() * Hl2).inverse();
        H.noalias() = Hx2.transpose() * (Hx2 - (Hl2 * (C2 * (Hl2.transpose() * Hx2))));
        gs_vector.noalias() = Hx2.transpose() * (b2 - (Hl2 * (C2 * (Hl2.transpose() * b2))));

        // Populate Gs and gs
        int GsCount2 = 0;
        for(size_t i1 = 0; i1 < numKeys; i1++) {
          gs.at(i1) = sub(gs_vector, 6*i1, 6*i1 + 6);

          for(size_t i2 = 0; i2 < numKeys; i2++) {
            if (i2 >= i1) {
              Gs.at(GsCount2) = H.block(6*i1, 6*i2, 6, 6);
              GsCount2++;
            }
          }
        }
      }

      // ==========================================================================================================
      if(linearizationThreshold >= 0){ // if we do not use selective relinearization we don't need to store these variables
        state_->calculatedHessian = true;
        state_->Gs = Gs;
        state_->gs = gs;
        state_->f = f;
      }

      return HessianFactor::shared_ptr(new HessianFactor(keys_, Gs, gs, f));
    }

    /**
     * Calculate the error of the factor.
     * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
     * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
     * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
     */
    virtual double error(const Values& values) const {
      if (this->active(values)) {
        double overallError=0;

        // Collect all poses (Cameras)
        std::vector<Pose3> cameraPoses;
        BOOST_FOREACH(const Key& k, keys_) {
          Pose3 cameraPose;
          if(body_P_sensor_)    {   cameraPose = values.at<Pose3>(k).compose(*body_P_sensor_);}
          else                         { cameraPose = values.at<Pose3>(k);}
          cameraPoses.push_back(cameraPose);

          if(0&& isDebug) {cameraPose.print("cameraPose = "); }
        }

        if(cameraPoses.size() < 2){ // if we have a single pose the corresponding factor is uninformative
          return 0.0;
        }

        bool retriangulate = decideIfTriangulate(cameraPoses, state_->cameraPosesTriangulation, retriangulationThreshold);

        if(retriangulate) {// we store the current poses used for triangulation
          state_->cameraPosesTriangulation = cameraPoses;
        }

        if (retriangulate) {
          // We triangulate the 3D position of the landmark
          try {
            state_->point = triangulatePoint3(cameraPoses, measured_, K_all_, rankTolerance);
            state_->degenerate = false;
            state_->cheiralityException = false;
          } catch( TriangulationUnderconstrainedException& e) {
            // if TriangulationUnderconstrainedException can be
            // 1) There is a single pose for triangulation - this should not happen because we checked the number of poses before
            // 2) The rank of the matrix used for triangulation is < 3: rotation-only, parallel cameras (or motion towards the landmark)
            // in the second case we want to use a rotation-only smart factor
            //std::cout << "Triangulation failed " << e.what() << std::endl; // point triangulated at infinity
            state_->degenerate = true;
            state_->cheiralityException = false;
          } catch( TriangulationCheiralityException& e) {
            // point is behind one of the cameras: can be the case of close-to-parallel cameras or may depend on outliers
            // we manage this case by either discarding the smart factor, or imposing a rotation-only constraint
            //std::cout << e.what() << std::end;
            state_->cheiralityException = true;
          }
        }

        if (!manageDegeneracy && (state_->cheiralityException || state_->degenerate) ){
          // if we don't want to manage the exceptions we discard the factor
          // std::cout << "In error evaluation: exception" << std::endl;
          return 0.0;
        }

        if (state_->cheiralityException || state_->degenerate){ // if we want to manage the exceptions with rotation-only factors
          state_->degenerate = true;
        }

        if(state_->degenerate){
          // return 0.0; // TODO: this maybe should be zero?
          for(size_t i = 0; i < measured_.size(); i++) {
            Pose3 pose = cameraPoses.at(i);
            PinholeCamera<CALIBRATION> camera(pose, *K_all_.at(i));
            if(i==0){ // first pose
              state_->point = camera.backprojectPointAtInfinity(measured_.at(i)); // 3D parametrization of point at infinity
            }
            Point2 reprojectionError(camera.projectPointAtInfinity(state_->point) - measured_.at(i));
            overallError +=  0.5 * noise_.at(i)->distance( reprojectionError.vector() );
            //overallError +=  reprojectionError.vector().norm();
          }
          return overallError;
        }
        else{
          for(size_t i = 0; i < measured_.size(); i++) {
            Pose3 pose = cameraPoses.at(i);
            PinholeCamera<CALIBRATION> camera(pose, *K_all_.at(i));

            try {
              Point2 reprojectionError(camera.project(state_->point) - measured_.at(i));
              //std::cout << "Reprojection error: " << reprojectionError << std::endl;
              overallError += 0.5 * noise_.at(i)->distance( reprojectionError.vector() );
              //overallError += reprojectionError.vector().norm();
            } catch ( CheiralityException& e) {
              std::cout << "Cheirality exception " << state_->ID << std::endl;
              exit(EXIT_FAILURE);
            }
          }
          return overallError;
        }
      } else { // else of active flag
        return 0.0;
      }
    }

    /** return the measurements */
    const Vector& measured() const {
      return measured_;
    }

    /** return the noise model */
    const SharedNoiseModel& noise() const {
      return noise_; 
    }

    /** return the landmark */
    boost::optional<Point3> point() const {
      return state_->point;
    }

    /** return the calibration object */
    inline const boost::shared_ptr<CALIBRATION> calibration() const {
      return K_all_;
    }

    /** return the calibration object */
    inline bool isDegenerate() const {
      return (state_->cheiralityException || state_->degenerate);
    }

    /** return verbosity */
    inline bool verboseCheirality() const { return verboseCheirality_; }

    /** return flag for throwing cheirality exceptions */
    inline bool throwCheirality() const { return throwCheirality_; }

  private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(K_all_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }

  };

} // \ namespace gtsam

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
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <vector>
#include <gtsam_unstable/geometry/triangulation.h>
#include <boost/optional.hpp>
#include <boost/assign.hpp>

namespace gtsam {

  /**
   * Structure for storing some state memory, used to speed up optimization
   * @addtogroup SLAM
   */
  class SmartProjectionFactorState {
  public:

    static int lastID;
    int ID;

    SmartProjectionFactorState() {
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

    // Jacobian representation (before Schur complement)
    Matrix Hx;
    Matrix Hl;
    Vector b;

    // C = Hl'Hl
    // Cinv = inv(Hl'Hl)
    // Matrix3 Cinv;
    // E = Hx'Hl
    // w = Hl'b
  };

  int SmartProjectionFactorState::lastID = 0;

  /**
   * The calibration is known here.
   * @addtogroup SLAM
   */
  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class SmartProjectionFactor: public NonlinearFactor {
  protected:

    // Keep a copy of measurement and calibration for I/O
    std::vector<Point2> measured_;                    ///< 2D measurement for each of the m views
    const SharedNoiseModel noise_;   ///< noise model used
    ///< (important that the order is the same as the keys that we use to create the factor)
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame
    boost::shared_ptr<SmartProjectionFactorState> state_;

    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)


  public:

    /// shorthand for base class type
    typedef NonlinearFactor Base;

    /// shorthand for this class
    typedef SmartProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// shorthand for smart projection factor state variable
    typedef boost::shared_ptr<SmartProjectionFactorState> SmartFactorStatePtr;

    /// Default constructor
    SmartProjectionFactor() : throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor
     * @param poseKeys is the set of indices corresponding to the cameras observing the same landmark
     * @param measured is the 2m dimensional location of the projection of a single landmark in the m views (the measurements)
     * @param model is the standard deviation (current version assumes that the uncertainty is the same for all views)
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    SmartProjectionFactor(std::vector<Key> poseKeys, // camera poses
        const std::vector<Point2> measured,          // pixel measurements
        const SharedNoiseModel& model,               // noise model (same for all measurements)
        const boost::shared_ptr<CALIBRATION>& K,     // calibration matrix (same for all measurements)
        boost::optional<POSE> body_P_sensor = boost::none,
        SmartFactorStatePtr state = SmartFactorStatePtr(new SmartProjectionFactorState())) :
          measured_(measured), noise_(model), K_(K), body_P_sensor_(body_P_sensor),
          state_(state), throwCheirality_(false), verboseCheirality_(false) {
      keys_.assign(poseKeys.begin(), poseKeys.end());
    }

    /**
     * Constructor with exception-handling flags
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2m dimensional location of the projection of a single landmark in the m views (the measurements)
     * @param model is the standard deviation (current version assumes that the uncertainty is the same for all views)
     * @param poseKeys is the set of indices corresponding to the cameras observing the same landmark
     * @param K shared pointer to the constant calibration
     * @param throwCheirality determines whether Cheirality exceptions are rethrown
     * @param verboseCheirality determines whether exceptions are printed for Cheirality
     * @param body_P_sensor is the transform from body to sensor frame  (default identity)
     */
    SmartProjectionFactor(std::vector<Key> poseKeys,
        const std::vector<Point2> measured,
        const SharedNoiseModel& model,
        const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none,
        SmartFactorStatePtr state = SmartFactorStatePtr(new SmartProjectionFactorState())) :
          measured_(measured), noise_(model), K_(K), body_P_sensor_(body_P_sensor),
          state_(state), throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {
    }

    /**
     * Constructor with exception-handling flags
     * @param model is the standard deviation (current version assumes that the uncertainty is the same for all views)
     * @param K shared pointer to the constant calibration
     */
    SmartProjectionFactor(const SharedNoiseModel& model, const boost::shared_ptr<CALIBRATION>& K, 
        boost::optional<POSE> body_P_sensor = boost::none,
        SmartFactorStatePtr state = SmartFactorStatePtr(new SmartProjectionFactorState())) :
        noise_(model), K_(K), body_P_sensor_(body_P_sensor), state_(state) {
    }

    /** Virtual destructor */
    virtual ~SmartProjectionFactor() {}

    /**
     * add a new measurement and pose key
     * @param measured is the 2m dimensional location of the projection of a single landmark in the m view (the measurement)
     * @param poseKey is the index corresponding to the camera observing the same landmark
     */
    void add(const Point2 measured, const Key poseKey) {
      measured_.push_back(measured);
      keys_.push_back(poseKey);
    }

    // This function decides whether a new triangulation is needed
    inline bool decideIfTriangulate(std::vector<Pose3> cameraPoses, const Values& values) const {
      // several calls to linearize will be done from the same linearization point, hence it is not needed to re-triangulate
      // Note that this is not yet "selecting linearization", that will come later, and we only check if the
      // current linearization is the "same" (up to tolerance) w.r.t. the last time we triangulated the point
      bool retriangulate = true;
      bool valuesEqualRetriangulation = true;
      double retriangulationThreshold = 1e-9;

      int poseCount = 0;
      BOOST_FOREACH(const Key& k, keys_) {
        Pose3 cameraPose;

        if(body_P_sensor_)
          cameraPose = values.at<Pose3>(k).compose(*body_P_sensor_);
        else
          cameraPose = values.at<Pose3>(k);

        if (!state_->cameraPosesTriangulation.empty()) {

          // TODO: are you sure that when using "add" the number of poses will be ok? (old linearization point will contain 1 pose less)
          if (!cameraPose.equals(state_->cameraPosesTriangulation[poseCount], retriangulationThreshold)) {
            valuesEqualRetriangulation = false;
          }

        } else {
          valuesEqualRetriangulation = false;
        }

        cameraPoses.push_back(cameraPose);
        poseCount++;
      }

      if (valuesEqualRetriangulation) {
        retriangulate = false;
      }

      return retriangulate;
    }

    // This function decides whether a new triangulation is needed
    //    bool decideIfLinearize(std::vector<Pose3> cameraPoses) const {
    //      // "selecting linearization"
    //      bool doLinearize = true;
    //      double linearizationThreshold = 1e-2;
    //
    //      Pose3 firstCameraPose;
    //      Pose3 firstCameraPoseOld;
    //
    //      for(size_t i = 0; i < cameraPoses.size(); i++) {
    //        Pose3 cameraPose = cameraPoses.at(i);
    //
    //        if (!state_->cameraPosesLinearization.empty()) { // if we have a previous linearization point
    //
    //          if(i==0){ // we store the initial pose, this is useful for selective re-linearization
    //            firstCameraPose = cameraPose;
    //            firstCameraPoseOld = state_->cameraPosesLinearization[i];
    //            continue;
    //          }
    //
    //          // we compare the poses in the frame of the first pose
    //          Pose3 localCameraPose = firstCameraPose.between(cameraPose);
    //          Pose3 localCameraPoseOld = firstCameraPoseOld.between(state_->cameraPosesLinearization[i]);
    //
    //          if (!localCameraPose.equals(localCameraPoseOld, linearizationThreshold)) {
    //            doLinearize = false;
    //          }
    //
    //        } else {
    //          doLinearize = false;
    //        }
    //      }
    //
    //      return doLinearize;
    //    }


    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "SmartProjectionFactor, z = ";
      BOOST_FOREACH(const Point2& p, measured_) {
        std::cout << "measurement, p = "<< p << std::endl;
      }
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
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
          && this->K_->equals(*e->K_, tol)
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
      std::vector<Index> js;
      std::vector<Matrix> Gs(numKeys*(numKeys+1)/2);
      std::vector<Vector> gs(numKeys);
      double f=0;

      // Collect all poses (Cameras)
      std::vector<Pose3> cameraPoses;

      bool retriangulate = true; // decideIfTriangulate(cameraPoses, values);

      BOOST_FOREACH(const Key& k, keys_) {
        Pose3 cameraPose;

        if(body_P_sensor_)
          cameraPose = values.at<Pose3>(k).compose(*body_P_sensor_);
        else
          cameraPose = values.at<Pose3>(k);

        cameraPoses.push_back(cameraPose);
      }

      if(retriangulate) {
        state_->cameraPosesTriangulation = cameraPoses;
      }

      if (retriangulate) {
        // We triangulate the 3D position of the landmark
        try {
          Point3 newPoint = triangulatePoint3(cameraPoses, measured_, *K_);
          // changeLinPoint = newPoint - state_->point;  // TODO: implement this check for the degenerate case
          state_->point = newPoint;
          state_->degenerate = false;
          state_->cheiralityException = false;
        } catch( TriangulationUnderconstrainedException& e) {
          // point is triangulated at infinity
          //std::cout << e.what() << std::end;
          state_->degenerate = true;
          state_->cheiralityException = false;
          dim_landmark = 2;
        } catch( TriangulationCheiralityException& e) {
          // point is behind one of the cameras, turn factor off by setting everything to 0
          //std::cout << e.what() << std::end;
          BOOST_FOREACH(gtsam::Matrix& m, Gs) m = zeros(6, 6);
          BOOST_FOREACH(Vector& v, gs) v = zero(6);
          //state_->cheiralityException = true; // TODO: Debug condition, uncomment when fixed
          //return HessianFactor::shared_ptr(new HessianFactor(keys_, Gs, gs, f)); // TODO: Debug condition, uncomment when fixed
          // TODO: this is a debug condition, should be removed the comment
        }
      }
      state_->degenerate = true; // TODO: this is a debug condition, should be removed
      dim_landmark = 2; // TODO: this is a debug condition, should be removed the comment

      if (!retriangulate && state_->cheiralityException) {
        BOOST_FOREACH(gtsam::Matrix& m, Gs) m = zeros(6, 6);
        BOOST_FOREACH(Vector& v, gs) v = zero(6);
        return HessianFactor::shared_ptr(new HessianFactor(keys_, Gs, gs, f));
      }
      if (!retriangulate && state_->degenerate) {
        dim_landmark = 2;
      }

      bool doLinearize = true; //= decideIfLinearize(cameraPoses);

      if (doLinearize) {
        state_->cameraPosesLinearization = cameraPoses;
      }

      if(!doLinearize){ // return the previous Hessian factor
        return HessianFactor::shared_ptr(new HessianFactor(keys_, state_->Gs, state_->gs, state_->f));
      }
      //otherwise redo linearization

      if (blockwise){
        // ==========================================================================================================
        std::cout << "Deprecated use of blockwise version. This is slower and no longer supported" << std::endl;
        blockwise = false;
        //        std::vector<Matrix> Hx(numKeys);
        //        std::vector<Matrix> Hl(numKeys);
        //        std::vector<Vector> b(numKeys);
        //
        //        for(size_t i = 0; i < measured_.size(); i++) {
        //          Pose3 pose = cameraPoses.at(i);
        //          PinholeCamera<CALIBRATION> camera(pose, *K_);
        //          b.at(i) = - ( camera.project(state_->point,Hx.at(i),Hl.at(i)) - measured_.at(i) ).vector();
        //          noise_-> WhitenSystem(Hx.at(i), Hl.at(i), b.at(i));
        //          f += b.at(i).squaredNorm();
        //        }
        //
        //        // Shur complement trick
        //
        //        // Allocate m^2 matrix blocks
        //        std::vector< std::vector<Matrix> > Hxl(keys_.size(), std::vector<Matrix>( keys_.size()));
        //
        //        // Allocate inv(Hl'Hl)
        //        Matrix3 C = zeros(3,3);
        //        for(size_t i1 = 0; i1 < keys_.size(); i1++) {
        //          C.noalias() += Hl.at(i1).transpose() * Hl.at(i1);
        //        }
        //
        //        Matrix3 Cinv = C.inverse(); //  this is very important: without eval, because of eigen aliasing the results will be incorrect
        //
        //        // Calculate sub blocks
        //        for(size_t i1 = 0; i1 < keys_.size(); i1++) {
        //          for(size_t i2 = 0; i2 < keys_.size(); i2++) {
        //            // we only need the upper triangular entries
        //            Hxl[i1][i2].noalias() = Hx.at(i1).transpose() * Hl.at(i1) * Cinv * Hl.at(i2).transpose();
        //          }
        //        }
        //        // Populate Gs and gs
        //        int GsCount = 0;
        //        for(size_t i1 = 0; i1 < numKeys; i1++) {
        //          gs.at(i1).noalias() = Hx.at(i1).transpose() * b.at(i1);
        //
        //          for(size_t i2 = 0; i2 < numKeys; i2++) {
        //            gs.at(i1).noalias() -= Hxl[i1][i2] * b.at(i2);
        //
        //            if (i2 == i1){
        //              Gs.at(GsCount).noalias() = Hx.at(i1).transpose() * Hx.at(i1) - Hxl[i1][i2] * Hx.at(i2);
        //              GsCount++;
        //            }
        //            if (i2 > i1) {
        //              Gs.at(GsCount).noalias() = - Hxl[i1][i2] * Hx.at(i2);
        //              GsCount++;
        //            }
        //          }
        //        }
      }

      if (blockwise == false){ // version with full matrix multiplication
        // ==========================================================================================================
        Matrix Hx2 = zeros(2 * numKeys, 6 * numKeys);
        Matrix Hl2 = zeros(2 * numKeys, dim_landmark);
        Vector b2 = zero(2 * numKeys);

        if(state_->degenerate){
          for(size_t i = 0; i < measured_.size(); i++) {
            Pose3 pose = cameraPoses.at(i);
            PinholeCamera<CALIBRATION> camera(pose, *K_);
            if(i==0){ // first pose
              state_->point = camera.backprojectPointAtInfinity(measured_.at(i)); // 3D parametrization of point at infinity
              // std::cout << "point_ " << state_->point<< std::endl;
            }
            Matrix Hxi, Hli;
            Vector bi = -( camera.projectPointAtInfinity(state_->point,Hxi,Hli) - measured_.at(i) ).vector();
            // std::cout << "Hxi \n" << Hxi<< std::endl;
            // std::cout << "Hli \n" << Hli<< std::endl;

            noise_-> WhitenSystem(Hxi, Hli, bi);
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
            PinholeCamera<CALIBRATION> camera(pose, *K_);
            Matrix Hxi, Hli;

            Vector bi;
            try {
              bi = -( camera.project(state_->point,Hxi,Hli) - measured_.at(i) ).vector();
            } catch ( CheiralityException& e) {
              std::cout << "Cheirality exception " << state_->ID << std::endl;
              exit(EXIT_FAILURE);
            }
            noise_-> WhitenSystem(Hxi, Hli, bi);

            Hx2.block( 2*i, 6*i, 2, 6 ) = Hxi;
            Hl2.block( 2*i, 0, 2, 3  ) = Hli;

            subInsert(b2,bi,2*i);
          }
        }

        state_->Hx = Hx2;
        state_->Hl = Hl2;
        state_->b = b2;

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
      state_->calculatedHessian = true;
      state_->Gs = Gs;
      state_->gs = gs;
      state_->f = f;

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

        // check if triangulation and linearization are actually needed
        bool retriangulate = true; //decideIfTriangulate(cameraPoses, values);

        BOOST_FOREACH(const Key& k, keys_) {
          Pose3 cameraPose;

          if(body_P_sensor_)
            cameraPose = values.at<Pose3>(k).compose(*body_P_sensor_);
          else
            cameraPose = values.at<Pose3>(k);

          cameraPoses.push_back(cameraPose);
        }

        if(retriangulate) {
          state_->cameraPosesTriangulation = cameraPoses;
        }

        // We triangulate the 3D position of the landmark
        if (retriangulate) {
          try {
            state_->point = triangulatePoint3(cameraPoses, measured_, *K_);
            state_->degenerate = false;
            state_->cheiralityException = false;
          } catch( TriangulationCheiralityException& e) {
            // std::cout << "TriangulationCheiralityException "  << std::endl;
            // point is behind one of the cameras, turn factor off by setting everything to 0
            //std::cout << e.what() << std::end;
            //state_->cheiralityException = true; // TODO: Debug condition, remove comment
            //return 0.0; // TODO: this is a debug condition, should be removed the comment
          } catch( TriangulationUnderconstrainedException& e) {
            // point is triangulated at infinity
            //std::cout << e.what() << std::endl;
            state_->degenerate = true;
            state_->cheiralityException = false;
          }
        }
        state_->degenerate = true; // TODO: this is a debug condition, should be removed

        if (!retriangulate && state_->cheiralityException) {
          return 0.0;
        }

        if(state_->degenerate){
          for(size_t i = 0; i < measured_.size(); i++) {
            Pose3 pose = cameraPoses.at(i);
            PinholeCamera<CALIBRATION> camera(pose, *K_);
            if(i==0){ // first pose
              state_->point = camera.backprojectPointAtInfinity(measured_.at(i)); // 3D parametrization of point at infinity
            }
            Point2 reprojectionError(camera.projectPointAtInfinity(state_->point) - measured_.at(i));
            overallError += noise_->distance( reprojectionError.vector() );
          }
          return overallError;
        }
        else{
          for(size_t i = 0; i < measured_.size(); i++) {
            Pose3 pose = cameraPoses.at(i);
            PinholeCamera<CALIBRATION> camera(pose, *K_);

            try {
              Point2 reprojectionError(camera.project(state_->point) - measured_.at(i));
              overallError += noise_->distance( reprojectionError.vector() );
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
      return K_;
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
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }

  };

} // \ namespace gtsam

/*
// Discarded version of decideIfTriangulate and decideIfLinearize
 *  This function decides whether a new triangulation and linearization is needed
bool decideIfLinearize(std::vector<Pose3> cameraPoses) {
  // Selective relinearization (check if new linearization is needed)
        Vector repErr_i;
        try {
          repErr_i = - ( camera.project(state_->point) - measured_.at(i) ).vector();
        } catch ( CheiralityException& e) {
          std::cout << "Cheirality exception " << state_->ID << std::endl;
          exit(EXIT_FAILURE);
        }
        noise_-> whitenInPlace(repErr_i);
        f += repErr_i.squaredNorm();

        Vector linRepErr;

        linRepErr = state_->Hx * changeLinPoses + state_->Hl * changeLinPoint.vector() - state_->b;

        double f_lin = linRepErr.squaredNorm();

        // Relinearization check
        if (state_->f - f_lin > 1e-7){
          double rho = (state_->f - f) / (state_->f - f_lin);
          if( fabs(rho) > 0.75 ){
            return HessianFactor::shared_ptr(new HessianFactor(keys_, state_->Gs, state_->gs, state_->f));
          }
        }
        else{
          return HessianFactor::shared_ptr(new HessianFactor(keys_, state_->Gs, state_->gs, state_->f));
        }



bool decideIfTriangulateAndLinearize(std::vector<Pose3> cameraPoses) {
      //      Vector changeLinPoses(numKeys*6);
      //      Point3 changeLinPoint;

      Pose3 firstCameraPose;
      Pose3 firstCameraPoseOld;

      int poseCount = 0;
      BOOST_FOREACH(const Key& k, keys_) {
        Pose3 cameraPose;

        if(body_P_sensor_)
          cameraPose = values.at<Pose3>(k).compose(*body_P_sensor_);
        else
          cameraPose = values.at<Pose3>(k);

        if (!state_->cameraPosesTriangulation.empty()) {

          if(poseCount==0){ // we store the initial pose, this is useful for selective re-linearization
            firstCameraPose = cameraPose;
            firstCameraPoseOld = state_->cameraPosesTriangulation[poseCount];
          }

          // TODO: are you sure that when using "add" the number of poses will be ok? (old linearization point will contain 1 pose less)
          if (!cameraPose.equals(state_->cameraPosesTriangulation[poseCount], retriangulationThreshold)) {
            valuesEqualRetriangulation = false;
            subInsert(changeLinPoses, Vector::Zero(6), 6*poseCount);
          }else{
            Vector changeLinPoses_i = Pose3::Logmap(state_->cameraPosesTriangulation[poseCount].between(cameraPose));
            subInsert(changeLinPoses, changeLinPoses_i, 6*poseCount);
          }
        } else {
          valuesEqualRetriangulation = false;
          subInsert(changeLinPoses, Vector::Zero(6), 6*poseCount);
        }

        cameraPoses.push_back(cameraPose);
        poseCount++;
      }
    }
    */


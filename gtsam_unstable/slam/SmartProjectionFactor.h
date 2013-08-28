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

  class SmartProjectionFactorState;

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
    boost::optional<Point3> point_;
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

    /// Default constructor
    SmartProjectionFactor() : throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2m dimensional location of the projection of a single landmark in the m views (the measurements)
     * @param model is the standard deviation (current version assumes that the uncertainty is the same for all views)
     * @param poseKeys is the set of indices corresponding to the cameras observing the same landmark
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    SmartProjectionFactor(const std::vector<Point2> measured, const SharedNoiseModel& model,
        std::vector<Key> poseKeys, const boost::shared_ptr<CALIBRATION>& K,
        boost::optional<LANDMARK> point = boost::none,
        boost::optional<POSE> body_P_sensor = boost::none,
        boost::shared_ptr<SmartProjectionFactorState> state = boost::shared_ptr<SmartProjectionFactorState>()) :
          measured_(measured), noise_(model), K_(K), point_(point), body_P_sensor_(body_P_sensor),
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
    SmartProjectionFactor(const std::vector<Point2> measured, const SharedNoiseModel& model,
        std::vector<Key> poseKeys, const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<LANDMARK> point = boost::none,
        boost::optional<POSE> body_P_sensor = boost::none,
        boost::shared_ptr<SmartProjectionFactorState> state = boost::shared_ptr<SmartProjectionFactorState>()) :
          measured_(measured), noise_(model), K_(K), point_(point), body_P_sensor_(body_P_sensor),
          state_(state), throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /**
     * Constructor with exception-handling flags
     * @param model is the standard deviation (current version assumes that the uncertainty is the same for all views)
     * @param K shared pointer to the constant calibration
     */
    SmartProjectionFactor(const SharedNoiseModel& model, const boost::shared_ptr<CALIBRATION>& K, 
        boost::optional<LANDMARK> point = boost::none,
        boost::optional<POSE> body_P_sensor = boost::none,
        boost::shared_ptr<SmartProjectionFactorState> state = boost::shared_ptr<SmartProjectionFactorState>()) :
        noise_(model), K_(K), point_(point), body_P_sensor_(body_P_sensor), state_(state) {
    }

    /** Virtual destructor */
    virtual ~SmartProjectionFactor() {}

    /// @return a deep copy of this factor
//    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
//      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
//          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * add
     * @param measured is the 2m dimensional location of the projection of a single landmark in the m view (the measurement)
     * @param poseKey is the index corresponding to the camera observing the same landmark
     */
    void add(const Point2 measured, const Key poseKey) {
      measured_.push_back(measured);
      keys_.push_back(poseKey);
    }

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

      bool blockwise = false;
      
      unsigned int numKeys = keys_.size();
      std::vector<Index> js;
      std::vector<Matrix> Gs(numKeys*(numKeys+1)/2);
      std::vector<Vector> gs(numKeys);
      double f=0;

      // Collect all poses (Cameras)
      std::vector<Pose3> cameraPoses;
      BOOST_FOREACH(const Key& k, keys_) {
        if(body_P_sensor_)
          cameraPoses.push_back(values.at<Pose3>(k).compose(*body_P_sensor_));
        else
          cameraPoses.push_back(values.at<Pose3>(k));
      }

      // We triangulate the 3D position of the landmark
      Point3 point;
      try {
          if (point_) {
            point = *point_;
          } else {
            point = triangulatePoint3(cameraPoses, measured_, *K_);
          }
      } catch( TriangulationCheiralityException& e) {
          // point is behind one of the cameras, turn factor off by setting everything to 0
          //std::cout << e.what() << std::end;
          BOOST_FOREACH(gtsam::Matrix& m, Gs) m = zeros(6, 6);
          BOOST_FOREACH(Vector& v, gs) v = zero(6);
          return HessianFactor::shared_ptr(new HessianFactor(keys_, Gs, gs, f));         
      }

      if (blockwise){
        // ==========================================================================================================
        std::vector<Matrix> Hx(numKeys);
        std::vector<Matrix> Hl(numKeys);
        std::vector<Vector> b(numKeys);

        for(size_t i = 0; i < measured_.size(); i++) {
          Pose3 pose = cameraPoses.at(i);
          PinholeCamera<CALIBRATION> camera(pose, *K_);
          b.at(i) = - ( camera.project(point,Hx.at(i),Hl.at(i)) - measured_.at(i) ).vector();
          noise_-> WhitenSystem(Hx.at(i), Hl.at(i), b.at(i));
          f += b.at(i).squaredNorm();
        }

        // Shur complement trick

        // Allocate m^2 matrix blocks
        std::vector< std::vector<Matrix> > Hxl(keys_.size(), std::vector<Matrix>( keys_.size()));

        // Allocate inv(Hl'Hl)
        Matrix3 C = zeros(3,3);
        for(size_t i1 = 0; i1 < keys_.size(); i1++) {
          C.noalias() += Hl.at(i1).transpose() * Hl.at(i1);
        }

        Matrix3 Cinv = C.inverse(); //  this is very important: without eval, because of eigen aliasing the results will be incorrect

        // Calculate sub blocks
        for(size_t i1 = 0; i1 < keys_.size(); i1++) {
          for(size_t i2 = 0; i2 < keys_.size(); i2++) {
            // we only need the upper triangular entries
            Hxl[i1][i2].noalias() = Hx.at(i1).transpose() * Hl.at(i1) * Cinv * Hl.at(i2).transpose();
          }
        }
        // Populate Gs and gs
        int GsCount = 0;
        for(size_t i1 = 0; i1 < numKeys; i1++) {
          gs.at(i1).noalias() = Hx.at(i1).transpose() * b.at(i1);

          for(size_t i2 = 0; i2 < numKeys; i2++) {
            gs.at(i1).noalias() -= Hxl[i1][i2] * b.at(i2);

            if (i2 == i1){
              Gs.at(GsCount).noalias() = Hx.at(i1).transpose() * Hx.at(i1) - Hxl[i1][i2] * Hx.at(i2);
              GsCount++;
            }
            if (i2 > i1) {
              Gs.at(GsCount).noalias() = - Hxl[i1][i2] * Hx.at(i2);
              GsCount++;
            }
          }
        }
      }

      if (blockwise == false){ // version with full matrix multiplication
        // ==========================================================================================================
        Matrix Hx2 = zeros(2 * numKeys, 6 * numKeys);
        Matrix Hl2 = zeros(2 * numKeys, 3);
        Vector b2 = zero(2 * numKeys);

        for(size_t i = 0; i < measured_.size(); i++) {
          Pose3 pose = cameraPoses.at(i);
          PinholeCamera<CALIBRATION> camera(pose, *K_);
          Matrix Hxi, Hli;
           Vector bi = -( camera.project(point,Hxi,Hli) - measured_.at(i) ).vector();

           noise_-> WhitenSystem(Hxi, Hli, bi);
           f += bi.squaredNorm();

           Hx2.block( 2*i, 6*i, 2, 6 ) = Hxi;
           Hl2.block( 2*i, 0, 2, 3  ) = Hli;

           subInsert(b2,bi,2*i);

        }

        // Shur complement trick
        Matrix H(6 * numKeys, 6 * numKeys);
        Matrix3 C2 = (Hl2.transpose() * Hl2).inverse();
        H = Hx2.transpose() * (Hx2 - (Hl2 * (C2 * (Hl2.transpose() * Hx2))));

        Vector gs_vector = Hx2.transpose() * (b2 - (Hl2 * (C2 * (Hl2.transpose() * b2))));


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
          if(body_P_sensor_)
            cameraPoses.push_back(values.at<Pose3>(k).compose(*body_P_sensor_));
          else
            cameraPoses.push_back(values.at<Pose3>(k));
        }

        // We triangulate the 3D position of the landmark
        Point3 point;
        try {
          if (point_) {
            point = *point_;
          } else {
            point = triangulatePoint3(cameraPoses, measured_, *K_);
          }
        } catch( TriangulationCheiralityException& e) {
            // point is behind one of the cameras, turn factor off by setting everything to 0
            //std::cout << e.what() << std::end;
            return 0.0;
        }

        for(size_t i = 0; i < measured_.size(); i++) {
          Pose3 pose = cameraPoses.at(i);
          PinholeCamera<CALIBRATION> camera(pose, *K_);

          Point2 reprojectionError(camera.project(point) - measured_.at(i));
          overallError += noise_->distance( reprojectionError.vector() );
        }
        return overallError;
      } else {
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

    /** return the noise landmark */
    boost::optional<Point3> point() const {
      return point_;
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

  /**
   * Structure for storing some state memory, used to speed up optimization
   * @addtogroup SLAM
   */
  class SmartProjectionFactorState {
  public:
    // Landmark key
    Key landmarkKey_;

    // Set of involved pose keys
    std::list<Key> poseKeys_;

    // Linearization point
    Values values_;

    // inv(C)
    Matrix3 Cinv_;

    // E
    // W
    // Hessian
    Matrix H_;

  };

} // \ namespace gtsam

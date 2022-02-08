/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  Pose2_Point2_Factor.h
 *  @author Frank Dellaert, Viorela Ila
 **/
#pragma once

#include <ostream>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam/base/numericalDerivative.h>

namespace gtsam {

  /**
   * A class for a measurement of a Point2 from a Pose2
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
  class Pose2_Point2_Factor: public NoiseModelFactor2<Pose2, Point2> {

  private:

    typedef Pose2_Point2_Factor This;
    typedef NoiseModelFactor2<Pose2, Point2> Base;

    Point2 measured_; /** The measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<Pose2_Point2_Factor> shared_ptr;

    /** default constructor - only use for serialization */
    Pose2_Point2_Factor() {}

    /** Constructor */
    Pose2_Point2_Factor(Key key1, Key key2, const Point2& measured,
        const SharedNoiseModel& model) :
      Base(model, key1, key2), measured_(measured) {
    }

    virtual ~Pose2_Point2_Factor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << "Pose2_Point2_Factor::print() NOT implemented yet" << std::endl;
      /*
      std::cout << s << "Pose2_Point2_Factor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      traits<T>::Print(measured_, "  measured: ");
      this->noiseModel_->print("  noise model: ");
      // */
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      /*
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol) && traits<T>::Equals(this->measured_, e->measured_, tol);
      // */
      std::cout << "Pose2_Point2_Factor::equals() NOT implemented yet" << std::endl;
      return false;
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
  Vector evaluateError(const Pose2& pose, const Point2& point, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none) const {
      
      auto err = [&] (const Pose2& this_pose, const Point2& this_point) {

        //Pose3 relate_pose = this_base.transform_pose_to(this_pose); //ominus????
        //Plane3d predicted_plane = Plane3d::Transform(this_plane, relate_pose);
        Point2 predicted_point = this_pose.transform_to(this_point);
        
        //return predicted_point.error(measured_);
        return measured_.localCoordinates(predicted_point);
      };

      if (H1) {
        *H1 = numericalDerivative21<Vector, Pose2, Point2>(err, pose, point);
      }
      if (H2) {
        *H2 = numericalDerivative22<Vector, Pose2, Point2>(err, pose, point);
      }
      return err(pose, point);
    }

    /** return the measured */
    const Point2& measured() const {
      return measured_;
    }

    /** number of variables attached to this factor */
    std::size_t size() const {
      return 2;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  }; // \class BetweenFactor

  /// traits
  //struct traits<Pose2_Point2_Factor> : public Testable<Pose2_Point2_Factor> {};

//=========================================== MH_Pose2_Point2_Factor ======================================================

  class MH_Pose2_Point2_Factor: public MHNoiseModelFactor_1toK<Pose2, Point2> {

  private:

    typedef MH_Pose2_Point2_Factor This;
    typedef MHNoiseModelFactor_1toK<Pose2, Point2> Base;

    Point2 measured_; //The measurement
    
    //[MH-E]:
    bool is_detachable_;

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<MH_Pose2_Point2_Factor> shared_ptr;

    MH_Pose2_Point2_Factor() {}

    MH_Pose2_Point2_Factor(std::list<Key> key1_key2_list, const Point2 measured,
        const SharedNoiseModel& model, const bool& is_detachable = false) :
      Base(model, key1_key2_list), measured_(measured), is_detachable_(is_detachable) { //[E]
    }

    virtual ~MH_Pose2_Point2_Factor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }


    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      //TODO: Have to loop through all the modes (NOT implemented yet)
      std::cout << "MH_Pose2_Point2_Factor::print() NOT implemented yet" << std::endl;
    }

    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      //TODO: Have to loop through all the modes (NOT implemented yet)
      std::cout << "MH_Pose2_Point2_Factor::equals() NOT implemented yet" << std::endl;
      return false; 
    }

  //[MH-A]: only replace measured_ with measured_arr_[mode_id]
  Vector evaluateSingleError(const Pose2& pose, const std::vector<Point2>& point_arr, const size_t& mode_id, boost::optional<std::vector<Matrix>&> H_arr = boost::none) const {
      
      const int& dim = 2;
     
      if (mode_id < point_arr.size()) { 
      
        auto err = [&] (const Pose2& this_pose, const Point2& this_point) {

          Point2 predicted_point = this_pose.transform_to(this_point);
        
          return measured_.localCoordinates(predicted_point);
        };

        if (H_arr) {
          (*H_arr)[0] = numericalDerivative21<Vector, Pose2, Point2>(err, pose, point_arr[mode_id]);
          for (size_t i = 1; i <= point_arr.size(); ++i) {
            if (i == mode_id + 1) {
              (*H_arr)[i] = numericalDerivative22<Vector, Pose2, Point2>(err, pose, point_arr[mode_id]);
            } else {
              (*H_arr)[i] = Matrix::Zero(dim, dim);
            }
          }
        }
        return err(pose, point_arr[mode_id]);

      } else { 
        // The detached case should output zeros for both error and H
        (*H_arr)[0] = Matrix::Zero(dim, 3);
        for (size_t i = 1; i <= point_arr.size(); ++i) {
          (*H_arr)[i] = Matrix::Zero(dim, dim);
        }
        
        return Vector::Zero(dim, 1);
      
      }
    }

    const Point2& measured() const {
      return measured_;
    }

    std::size_t size() const {
      return keys_.size();
    }

  private:

    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int ) { //version
      ar & boost::serialization::make_nvp("MHNoiseModelFactor_1toK",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_); 
    }
  }; // class MH_Pose2_Point2_Factor

//=========================================== END MH_Pose2_Point2_Factor ======================================================

} /// namespace gtsam

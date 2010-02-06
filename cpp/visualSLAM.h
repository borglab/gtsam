/*
 * visualSLAM.h
 *
 *  Created on: Jan 14, 2010
 *      Author: Richard Roberts and Chris Beall
 */

#pragma once

#include "Key.h"
#include "Pose3.h"
#include "Point3.h"
#include "NonlinearFactorGraph.h"
#include "Cal3_S2.h"
#include "Point2.h"
#include "SimpleCamera.h"
#include "TupleConfig.h"
#include "NonlinearEquality.h"

namespace gtsam { namespace visualSLAM {

  /**
   * Typedefs that make up the visualSLAM namespace.
   */
  typedef TypedSymbol<Pose3,'x'> PoseKey;
  typedef TypedSymbol<Point3,'l'> PointKey;
  typedef PairConfig<PoseKey, Pose3, PointKey, Point3> Config;
  typedef NonlinearFactorGraph<Config> Graph;
  typedef NonlinearEquality<Config, PoseKey, Pose3> PoseConstraint;
  typedef NonlinearEquality<Config, PointKey, Point3> PointConstraint;


  /**
   * Non-linear factor for a constraint derived from a 2D measurement,
   * i.e. the main building block for visual SLAM.
   */
  template <class Cfg=Config>
  class GenericProjectionFactor : public NonlinearFactor2<Cfg, PoseKey, Pose3, PointKey, Point3>, Testable<GenericProjectionFactor<Cfg> > {
  private:

    // Keep a copy of measurement and calibration for I/O
    Point2 z_;
    boost::shared_ptr<Cal3_S2> K_;

  public:

    // shorthand for base class type
    typedef NonlinearFactor2<Cfg, PoseKey, Pose3, PointKey, Point3> Base;

    // shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<GenericProjectionFactor> shared_ptr;

    /**
     * Default constructor
     */
    GenericProjectionFactor() : K_(new Cal3_S2(444, 555, 666, 777, 888)) {}

    /**
     * Constructor
     * @param z is the 2 dimensional location of point in image (the measurement)
     * @param sigma is the standard deviation
     * @param cameraFrameNumber is basically the frame number
     * @param landmarkNumber is the index of the landmark
     * @param K the constant calibration
     */
    GenericProjectionFactor(const Point2& z,
					const SharedGaussian& model, PoseKey j_pose,
					PointKey j_landmark, const shared_ptrK& K) :
				z_(z), K_(K), Base(model, j_pose, j_landmark) {
			}

    /**
     * print
     * @param s optional string naming the factor
     */
    void print(const std::string& s = "ProjectionFactor") const {
        Base::print(s);
        z_.print(s + ".z");
    }

    /**
     * equals
     */
    bool equals(const GenericProjectionFactor<Cfg>& p, double tol = 1e-9) const {
        return Base::equals(p, tol) && z_.equals(p.z_, tol)
                  && K_->equals(*p.K_, tol);
    }

    //    /** h(x) */
    //    Point2 predict(const Pose3& pose, const Point3& point) const {
    //      return SimpleCamera(*K_, pose).project(point);
    //    }

    /** h(x)-z */
    Vector evaluateError(const Pose3& pose, const Point3& point,
        boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
      SimpleCamera camera(*K_, pose);
      if (H1) *H1=Dproject_pose(camera,point);
      if (H2) *H2=Dproject_point(camera,point);
      Point2 reprojectionError(project(camera, point) - z_);
      return reprojectionError.vector();
    }

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      //ar & BOOST_SERIALIZATION_NVP(key1_);
      //ar & BOOST_SERIALIZATION_NVP(key2_);
      ar & BOOST_SERIALIZATION_NVP(z_);
      ar & BOOST_SERIALIZATION_NVP(K_);
    }
  };

  typedef GenericProjectionFactor<Config> ProjectionFactor;




  //  /**
  //   * Non-linear factor graph for visual SLAM
  //   */
  //  class VSLAMGraph : public NonlinearFactorGraph<VSLAMConfig>{
  //
  //  public:
  //
  //    /** default constructor is empty graph */
  //    VSLAMGraph() {}
  //
  //    /**
  //     * print out graph
  //     */
  //    void print(const std::string& s = "") const {
  //      NonlinearFactorGraph<VSLAMConfig>::print(s);
  //    }
  //
  //    /**
  //     * equals
  //     */
  //    bool equals(const VSLAMGraph& p, double tol=1e-9) const {
  //      return NonlinearFactorGraph<VSLAMConfig>::equals(p, tol);
  //    }
  //
  //    /**
  //     *  Add a constraint on a landmark (for now, *must* be satisfied in any Config)
  //     *  @param j index of landmark
  //     *  @param p to which point to constrain it to
  //     */
  //    void addLandmarkConstraint(int j, const Point3& p = Point3());
  //
  //    /**
  //     *  Add a constraint on a camera (for now, *must* be satisfied in any Config)
  //     *  @param j index of camera
  //     *  @param p to which pose to constrain it to
  //     */
  //    void addCameraConstraint(int j, const Pose3& p = Pose3());
  //
  //  private:
  //    /** Serialization function */
  //    friend class boost::serialization::access;
  //    template<class Archive>
  //    void serialize(Archive & ar, const unsigned int version) {}
  //  };

} } // namespaces

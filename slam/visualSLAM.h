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
  typedef LieConfig<PoseKey, Pose3> PoseConfig;
  typedef LieConfig<PointKey, Point3> PointConfig;
  typedef TupleConfig2<PoseConfig, PointConfig> Config;

  typedef NonlinearEquality<Config, PoseKey, Pose3> PoseConstraint;
  typedef NonlinearEquality<Config, PointKey, Point3> PointConstraint;


  /**
   * Non-linear factor for a constraint derived from a 2D measurement,
   * i.e. the main building block for visual SLAM.
   */
  template <class Cfg=Config, class LmK=PointKey, class PosK=PoseKey>
  class GenericProjectionFactor : public NonlinearFactor2<Cfg, PosK, Pose3, LmK, Point3>, Testable<GenericProjectionFactor<Cfg, LmK, PosK> > {
  protected:

    // Keep a copy of measurement and calibration for I/O
    Point2 z_;
    boost::shared_ptr<Cal3_S2> K_;

  public:

    // shorthand for base class type
    typedef NonlinearFactor2<Cfg, PosK, Pose3, LmK, Point3> Base;

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
					const SharedGaussian& model, PosK j_pose,
					LmK j_landmark, const shared_ptrK& K) :
						Base(model, j_pose, j_landmark), z_(z), K_(K) {
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
    bool equals(const GenericProjectionFactor<Cfg, LmK, PosK>& p, double tol = 1e-9) const {
        return Base::equals(p, tol) && this->z_.equals(p.z_, tol)
                  && this->K_->equals(*p.K_, tol);
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

  // Typedef for general use
  typedef GenericProjectionFactor<Config, PointKey, PoseKey> ProjectionFactor;

  /**
	 * Non-linear factor graph for vanilla visual SLAM
	 */
	class Graph: public NonlinearFactorGraph<Config> {

	public:

		/** default constructor is empty graph */
		Graph() {
		}

		/** print out graph */
		void print(const std::string& s = "") const {
			NonlinearFactorGraph<Config>::print(s);
		}

		/** equals */
		bool equals(const Graph& p, double tol = 1e-9) const {
			return NonlinearFactorGraph<Config>::equals(p, tol);
		}

		/**
		 *  Add a measurement
		 *  @param j index of camera
		 *  @param p to which pose to constrain it to
		 */
		void addMeasurement(const Point2& z, const SharedGaussian& model,
				PoseKey i, PointKey j, const shared_ptrK& K) {
			boost::shared_ptr<ProjectionFactor> factor(new ProjectionFactor(z, model, i, j, K));
			push_back(factor);
		}

		/**
		 *  Add a constraint on a pose (for now, *must* be satisfied in any Config)
		 *  @param j index of camera
		 *  @param p to which pose to constrain it to
		 */
		void addPoseConstraint(int j, const Pose3& p = Pose3()) {
			boost::shared_ptr<PoseConstraint> factor(new PoseConstraint(j, p));
			push_back(factor);
		}

		/**
		 *  Add a constraint on a point (for now, *must* be satisfied in any Config)
		 *  @param j index of landmark
		 *  @param p to which point to constrain it to
		 */
		void addPointConstraint(int j, const Point3& p = Point3()) {
			boost::shared_ptr<PointConstraint> factor(new PointConstraint(j, p));
			push_back(factor);
		}

	}; // Graph

} } // namespaces

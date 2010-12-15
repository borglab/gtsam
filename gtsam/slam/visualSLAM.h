/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * visualSLAM.h
 *
 *  Created on: Jan 14, 2010
 *      Author: Richard Roberts and Chris Beall
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/TupleValues.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

namespace gtsam {

	/**
	 * Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here.
	 * i.e. the main building block for visual SLAM.
	 */
	template <class CFG, class LMK, class POSK>
	class GenericProjectionFactor : public NonlinearFactor2<CFG, POSK, LMK>, Testable<GenericProjectionFactor<CFG, LMK, POSK> > {
	protected:

		// Keep a copy of measurement and calibration for I/O
		Point2 z_;
		boost::shared_ptr<Cal3_S2> K_;

	public:

		// shorthand for base class type
		typedef NonlinearFactor2<CFG, POSK, LMK> Base;

		// shorthand for a smart pointer to a factor
		typedef boost::shared_ptr<GenericProjectionFactor<CFG, LMK, POSK> > shared_ptr;

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
				const SharedGaussian& model, POSK j_pose,
				LMK j_landmark, const shared_ptrK& K) :
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
		bool equals(const GenericProjectionFactor<CFG, LMK, POSK>& p, double tol = 1e-9) const {
			return Base::equals(p, tol) && this->z_.equals(p.z_, tol)
					&& this->K_->equals(*p.K_, tol);
		}



		/** h(x)-z */
		Vector evaluateError(const Pose3& pose, const Point3& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			SimpleCamera camera(*K_, pose);
			Point2 reprojectionError(camera.project(point, H1, H2) - z_);
			return reprojectionError.vector();
		}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(z_);
			ar & BOOST_SERIALIZATION_NVP(K_);
		}
	};


	namespace visualSLAM {

  /**
   * Typedefs that make up the visualSLAM namespace.
   */
  typedef TypedSymbol<Pose3,'x'> PoseKey;
  typedef TypedSymbol<Point3,'l'> PointKey;
  typedef LieValues<PoseKey> PoseValues;
  typedef LieValues<PointKey> PointValues;
  typedef TupleValues2<PoseValues, PointValues> Values;
  typedef boost::shared_ptr<Values> shared_values;

  typedef NonlinearEquality<Values, PoseKey> PoseConstraint;
  typedef NonlinearEquality<Values, PointKey> PointConstraint;
  typedef PriorFactor<Values, PoseKey> PosePrior;
  typedef PriorFactor<Values, PointKey> PointPrior;
  
  // Typedef for general use
  typedef GenericProjectionFactor<Values, PointKey, PoseKey> ProjectionFactor;

  /**
   * Non-linear factor graph for vanilla visual SLAM
   */
  class Graph: public NonlinearFactorGraph<Values> {

  public:

    typedef boost::shared_ptr<Graph> shared_graph;

    /** default constructor is empty graph */
    Graph() {
    }

    /** print out graph */
    void print(const std::string& s = "") const {
      NonlinearFactorGraph<Values>::print(s);
    }

    /** equals */
    bool equals(const Graph& p, double tol = 1e-9) const {
      return NonlinearFactorGraph<Values>::equals(p, tol);
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
     *  Add a constraint on a pose (for now, *must* be satisfied in any Values)
     *  @param j index of camera
     *  @param p to which pose to constrain it to
     */
    void addPoseConstraint(int j, const Pose3& p = Pose3()) {
      boost::shared_ptr<PoseConstraint> factor(new PoseConstraint(j, p));
      push_back(factor);
    }

    /**
     *  Add a constraint on a point (for now, *must* be satisfied in any Values)
     *  @param j index of landmark
     *  @param p to which point to constrain it to
     */
    void addPointConstraint(int j, const Point3& p = Point3()) {
      boost::shared_ptr<PointConstraint> factor(new PointConstraint(j, p));
      push_back(factor);
    }

    /**
     *  Add a prior on a pose
     *  @param j index of camera
     *  @param p to which pose to constrain it to
     *  @param model uncertainty model of this prior
     */
    void addPosePrior(int j, const Pose3& p = Pose3(), const SharedGaussian& model = noiseModel::Unit::Create(1)) {
      boost::shared_ptr<PosePrior> factor(new PosePrior(j, p, model));
      push_back(factor);
    }

    /**
     *  Add a prior on a landmark
     *  @param j index of landmark
     *  @param model uncertainty model of this prior
     */
    void addPointPrior(int j, const Point3& p = Point3(), const SharedGaussian& model = noiseModel::Unit::Create(1)) {
      boost::shared_ptr<PointPrior> factor(new PointPrior(j, p, model));
      push_back(factor);
    }

  }; // Graph

  // Optimizer
  typedef NonlinearOptimizer<Graph, Values> Optimizer;

} } // namespaces

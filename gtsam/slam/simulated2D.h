/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    simulated2D.h
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

// \callgraph
#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/TupleValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

// \namespace

namespace gtsam {

	namespace simulated2D {

		// Simulated2D robots have no orientation, just a position
		typedef TypedSymbol<Point2, 'x'> PoseKey;
		typedef TypedSymbol<Point2, 'l'> PointKey;
		typedef LieValues<PoseKey> PoseValues;
		typedef LieValues<PointKey> PointValues;

		/**
		 *  Custom Values class that holds poses and points
		 */
		class Values: public TupleValues2<PoseValues, PointValues> {
		public:
			typedef TupleValues2<PoseValues, PointValues> Base;  ///< base class
			typedef boost::shared_ptr<Point2> sharedPoint;  ///< shortcut to shared Point type

			/// Constructor
			Values() {
			}

			/// Copy constructor
			Values(const Base& base) :
					Base(base) {
			}

			/// Insert a pose
			void insertPose(const simulated2D::PoseKey& i, const Point2& p) {
				insert(i, p);
			}

			/// Insert a point
			void insertPoint(const simulated2D::PointKey& j, const Point2& p) {
				insert(j, p);
			}

			/// Number of poses
			int nrPoses() const {
				return this->first_.size();
			}

			/// Number of points
			int nrPoints() const {
				return this->second_.size();
			}

			/// Return pose i
			sharedPoint pose(const simulated2D::PoseKey& i) {
				return sharedPoint(new Point2((*this)[i]));
			}

			/// Return point j
			sharedPoint point(const simulated2D::PointKey& j) {
				return sharedPoint(new Point2((*this)[j]));
			}
		};

		/// Prior on a single pose
		inline Point2 prior(const Point2& x) {
			return x;
		}

		/// Prior on a single pose, optionally returns derivative
		Point2 prior(const Point2& x, boost::optional<Matrix&> H = boost::none);

		/// odometry between two poses
		inline Point2 odo(const Point2& x1, const Point2& x2) {
			return x2 - x1;
		}

		/// odometry between two poses, optionally returns derivative
		Point2 odo(const Point2& x1, const Point2& x2, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none);

		/// measurement between landmark and pose
		inline Point2 mea(const Point2& x, const Point2& l) {
			return l - x;
		}

		/// measurement between landmark and pose, optionally returns derivative
		Point2 mea(const Point2& x, const Point2& l, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none);

		/**
		 *  Unary factor encoding a soft prior on a vector
		 */
		template<class CFG = Values, class KEY = PoseKey>
		class GenericPrior: public NonlinearFactor1<CFG, KEY> {
		public:
			typedef NonlinearFactor1<CFG, KEY> Base;  ///< base class
			typedef boost::shared_ptr<GenericPrior<CFG, KEY> > shared_ptr;
			typedef typename KEY::Value Pose; ///< shortcut to Pose type

			Pose z_; ///< prior mean

			/// Create generic prior
			GenericPrior(const Pose& z, const SharedNoiseModel& model, const KEY& key) :
					NonlinearFactor1<CFG, KEY>(model, key), z_(z) {
			}

			/// Return error and optional derivative
			Vector evaluateError(const Pose& x, boost::optional<Matrix&> H =
					boost::none) const {
				return (prior(x, H) - z_).vector();
			}

		private:

			/// Default constructor
			GenericPrior() {
			}

			/// Serialization function
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
				ar & BOOST_SERIALIZATION_NVP(z_);
			}
		};

		/**
		 * Binary factor simulating "odometry" between two Vectors
		 */
		template<class CFG = Values, class KEY = PoseKey>
		class GenericOdometry: public NonlinearFactor2<CFG, KEY, KEY> {
		public:
			typedef NonlinearFactor2<CFG, KEY, KEY> Base; ///< base class
			typedef boost::shared_ptr<GenericOdometry<CFG, KEY> > shared_ptr;
			typedef typename KEY::Value Pose; ///< shortcut to Pose type

			Pose z_; ///< odometry measurement

			/// Create odometry
			GenericOdometry(const Pose& z, const SharedNoiseModel& model,
					const KEY& i1, const KEY& i2) :
					NonlinearFactor2<CFG, KEY, KEY>(model, i1, i2), z_(z) {
			}

			/// Evaluate error and optionally return derivatives
			Vector evaluateError(const Pose& x1, const Pose& x2,
					boost::optional<Matrix&> H1 = boost::none,
					boost::optional<Matrix&> H2 = boost::none) const {
				return (odo(x1, x2, H1, H2) - z_).vector();
			}

		private:

			/// Default constructor
			GenericOdometry() {
			}

			/// Serialization function
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
				ar & BOOST_SERIALIZATION_NVP(z_);
			}
		};

		/**
		 * Binary factor simulating "measurement" between two Vectors
		 */
		template<class CFG = Values, class XKEY = PoseKey, class LKEY = PointKey>
		class GenericMeasurement: public NonlinearFactor2<CFG, XKEY, LKEY> {
		public:
			typedef NonlinearFactor2<CFG, XKEY, LKEY> Base;  ///< base class
			typedef boost::shared_ptr<GenericMeasurement<CFG, XKEY, LKEY> > shared_ptr;
			typedef typename XKEY::Value Pose; ///< shortcut to Pose type
			typedef typename LKEY::Value Point; ///< shortcut to Point type

			Point z_; ///< Measurement

			/// Create measurement factor
			GenericMeasurement(const Point& z, const SharedNoiseModel& model,
					const XKEY& i, const LKEY& j) :
					NonlinearFactor2<CFG, XKEY, LKEY>(model, i, j), z_(z) {
			}

			/// Evaluate error and optionally return derivatives
			Vector evaluateError(const Pose& x1, const Point& x2,
					boost::optional<Matrix&> H1 = boost::none,
					boost::optional<Matrix&> H2 = boost::none) const {
				return (mea(x1, x2, H1, H2) - z_).vector();
			}

		private:

			/// Default constructor
			GenericMeasurement() {
			}

			/// Serialization function
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
				ar & BOOST_SERIALIZATION_NVP(z_);
			}
		};

		/** Typedefs for regular use */
		typedef GenericPrior<Values, PoseKey> Prior;
		typedef GenericOdometry<Values, PoseKey> Odometry;
		typedef GenericMeasurement<Values, PoseKey, PointKey> Measurement;

	} // namespace simulated2D
} // namespace gtsam

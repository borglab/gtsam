/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BearingRangeFactor.h
 * @date Apr 1, 2010
 * @author Kai Ni
 * @brief a single factor contains both the bearing and the range to prevent handle to pair bearing and range factors
 */

#pragma once

#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

	/**
	 * Binary factor for a bearing measurement
	 * @addtogroup SLAM
	 */
	template<class POSE, class POINT, class ROTATION = typename POSE::Rotation>
	class BearingRangeFactor: public NoiseModelFactor2<POSE, POINT> {
	private:

		typedef POSE Pose;
		typedef ROTATION Rot;
		typedef POINT Point;

		typedef BearingRangeFactor<POSE, POINT> This;
		typedef NoiseModelFactor2<POSE, POINT> Base;

		// the measurement
		Rot measuredBearing_;
		double measuredRange_;

		/** concept check by type */
		GTSAM_CONCEPT_TESTABLE_TYPE(Rot)
		GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(Pose, Point)
		GTSAM_CONCEPT_POSE_TYPE(Pose)

	public:

		BearingRangeFactor() {} /* Default constructor */
		BearingRangeFactor(Key poseKey, Key pointKey, const Rot& measuredBearing, const double measuredRange,
				const SharedNoiseModel& model) :
					Base(model, poseKey, pointKey), measuredBearing_(measuredBearing), measuredRange_(measuredRange) {
		}

		virtual ~BearingRangeFactor() {}

		/// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
		      gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

	  /** Print */
	  virtual void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
	    std::cout << s << "BearingRangeFactor("
	    		<< keyFormatter(this->key1()) << ","
	    		<< keyFormatter(this->key2()) << ")\n";
	    measuredBearing_.print("measured bearing: ");
	    std::cout << "measured range: " << measuredRange_ << std::endl;
	    this->noiseModel_->print("noise model:\n");
	  }

		/** equals */
		virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
			const This *e =	dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) &&
					fabs(this->measuredRange_ - e->measuredRange_) < tol &&
					this->measuredBearing_.equals(e->measuredBearing_, tol);
		}

		/** h(x)-z -> between(z,h(x)) for Rot manifold */
		Vector evaluateError(const Pose& pose, const Point& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			Matrix H11, H21, H12, H22;
			boost::optional<Matrix&> H11_ = H1 ? boost::optional<Matrix&>(H11) : boost::optional<Matrix&>();
			boost::optional<Matrix&> H21_ = H1 ? boost::optional<Matrix&>(H21) : boost::optional<Matrix&>();
			boost::optional<Matrix&> H12_ = H2 ? boost::optional<Matrix&>(H12) : boost::optional<Matrix&>();
			boost::optional<Matrix&> H22_ = H2 ? boost::optional<Matrix&>(H22) : boost::optional<Matrix&>();

			Rot y1 = pose.bearing(point, H11_, H12_);
			Vector e1 = Rot::Logmap(measuredBearing_.between(y1));

			double y2 = pose.range(point, H21_, H22_);
			Vector e2 = Vector_(1, y2 - measuredRange_);

			if (H1) *H1 = gtsam::stack(2, &H11, &H21);
			if (H2) *H2 = gtsam::stack(2, &H12, &H22);
			return concatVectors(2, &e1, &e2);
		}

		/** return the measured */
		const std::pair<Rot, double> measured() const {
			return std::make_pair(measuredBearing_, measuredRange_);
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NoiseModelFactor2",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(measuredBearing_);
			ar & BOOST_SERIALIZATION_NVP(measuredRange_);
		}
	}; // BearingRangeFactor

} // namespace gtsam

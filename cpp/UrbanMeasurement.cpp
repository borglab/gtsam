/**
 * @file    UrbanMeasurement.cpp
 * @brief   A non-linear factor specialized to the Visual SLAM problem
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#include "UrbanMeasurement.h"
#include "Pose3.h"

using namespace std;

// Notation:
// ph = phi = roll
// th = theta = pitch
// ps = psi = yaw
// JC:  Need to confirm that RQ does the extraction
// that I think it's doing.  That should shake out with
// test data.
#define _GET_TEMPORARIES				\
	Vector rpy = RQ(pose.rotation().matrix());	\
	double dx = p.x() - pose.translation().x();	\
	double dy = p.y() - pose.translation().y();	\
	double s23 = (*sensorMatrix)(2, 3);		\
	double cps = cos(rpy[2]);			\
	double sps = sin(rpy[2]);			\
	double secph = 1/cos(rpy[0]);			\
	double tph = tan(rpy[0]);			\
	double sph = sin(rpy[0]);			\
	double sth = sin(rpy[1]);			\
	double cth = cos(rpy[1]);			\
	double tth = tan(rpy[1]);			\
	double secth = 1/cth

namespace gtsam {

	/* ************************************************************************* */

	// JC:  Clearly there is a lot of code overlap (and redundant calculations) between 
	// this and the calculation of the relevant jacobians.  Need to look at cleaning
	// this up when I have a better grasp of the larger implications.  I don't think
	// there's any reason we can't just merge transform_to and the two derivative
	// functions into a single function to remove the overlap.

	Point2 transform_to(const boost::shared_ptr<const Matrix> &sensorMatrix, 
			    const Pose3& pose, const Point2& p) {
		_GET_TEMPORARIES;
		
		return Point2(cth*dy*sps+(-s23*secph+dy*sps*sth+dx*sps*tph)*tth
			                +cps*(cth*dx+dx*sth*tth-dy*tph*tth),
			      secph*(cps*dy+s23*sph-dx*sps));

		// JC: This code still here for my reference.  
#if 0
		// create a 3D point at our height (Z is assumed up)
		Point3 global3D(p.x(),p.y(),pose.translation().z());
		// transform into local 3D point
		Point3 local3D = transform_to(pose,global3D);
		// take x and y as the local measurement
		Point2 local2D(local3D.x(),local3D.y());
		return local2D;
#endif
	}

	/* ************************************************************************* */
	Matrix Dtransform_to1(const boost::shared_ptr<const Matrix> &sensorMatrix, 
			      const Pose3& pose, const Point2& p) {
		_GET_TEMPORARIES;
		Matrix ret(2, 6);
		ret(0, 0) = (-cps*secth-sps*tph*tth);
                ret(0, 1) = (-secth*sps+cps*tph*tth);
		ret(0, 2) = 0;
		ret(0, 3) = -((secph*secph*(cps*dy+s23*sph-dx*sps)*tth));
                ret(0, 4) = (-((secth*(secth*(s23*secph+(cps*dy-dx*sps)*tph)
				       +(-cps*dx-dy*sps)*tth))));
		ret(0, 5) = ((cth*(cps*dy-dx*sps)+(cps*(dy*sth+dx*tph)
						   +sps*(-dx*sth+dy*tph))*tth));
		
                ret(1, 0) = secph*sps;
		ret(1, 1) = -cps*secph;
		ret(1, 2) = 0;
                ret(1, 3) = secph*(s23*secph+(cps*dy-dx*sps)*tph);
		ret(1, 4) = 0;
                ret(1, 5) = secph*(-cps*dx-dy*sps);
		return ret;
	}

	/* ************************************************************************* */
	Matrix Dtransform_to2(const boost::shared_ptr<const Matrix> &sensorMatrix, 
			      const Pose3& pose, const Point2& p) {
		_GET_TEMPORARIES;
		Matrix ret(2, 2);
		ret(0, 0) = cps*secth+sps*tph*tth;
                ret(0, 1) = secth*sps-cps*tph*tth;
                ret(1, 0) = -secph*sps;
		ret(1, 1) = cps*secph;
		return zeros(2, 2);
	}

#undef _GET_TEMPORARIES
	
	/* ************************************************************************* */
	UrbanMeasurement::UrbanMeasurement() {
		/// Arbitrary values

		// JC: This should cause use of a default-constructed measurement to segfault.
		// Not entirely sure as to the desired behaviour when uninitialized data is used.
		// If you're crashing because of this and you didn't expect to, sorry.  :)
		sensorMatrix_.reset(); 

		robotPoseNumber_ = 111;
		landmarkNumber_ = 222;
		robotPoseName_ = symbol('x', robotPoseNumber_);
		landmarkName_ = symbol('l', landmarkNumber_);
		keys_.push_back(robotPoseName_);
		keys_.push_back(landmarkName_);
	}

	/* ************************************************************************* */
	UrbanMeasurement::UrbanMeasurement(const boost::shared_ptr<const Matrix> &sensorMatrix, 
					   const Point2& z, double sigma, int i,
					   int j) :
		UrbanFactor(z.vector(), sigma) {
		
		sensorMatrix_ = sensorMatrix;
		robotPoseNumber_ = i;
		landmarkNumber_ = j;
		robotPoseName_ = symbol('x', robotPoseNumber_);
		landmarkName_ = symbol('l', landmarkNumber_);
		keys_.push_back(robotPoseName_);
		keys_.push_back(landmarkName_);
	}

	/* ************************************************************************* */
	void UrbanMeasurement::print(const std::string& s) const {
		printf("%s %s %s\n", s.c_str(), robotPoseName_.c_str(),
				landmarkName_.c_str());
		gtsam::print(this->z_, s + ".z");
	}

	/* ************************************************************************* */
	bool UrbanMeasurement::equals(const UrbanMeasurement& p, double tol) const {
		if (&p == NULL) return false;
		if (robotPoseNumber_ != p.robotPoseNumber_ || landmarkNumber_
				!= p.landmarkNumber_) return false;
		if (!equal_with_abs_tol(this->z_, p.z_, tol)) return false;
		return true;
	}

	/* ************************************************************************* */
	Vector UrbanMeasurement::error_vector(const UrbanConfig& x) const {
		Pose3 pose = x.robotPose(robotPoseNumber_);
		Point2 landmark = x.landmarkPoint(landmarkNumber_);
		// Right-hand-side b = (z - h(x))/sigma
		Point2 hx = transform_to(sensorMatrix_, pose,landmark);
		return (z_ - hx.vector());
	}
	
	/* ************************************************************************* */
	GaussianFactor::shared_ptr UrbanMeasurement::linearize(const UrbanConfig& config) const {
		Pose3 pose = config.robotPose(robotPoseNumber_);
		Point2 landmark = config.landmarkPoint(landmarkNumber_);

		// JC: Things I don't yet understand:
		//
		// What should the keys be?  Presumably I just need to match a convention.
		//
		// Should the jacobians be premultiplied at GuassianFactor
		// construction time?
		//
		// Why is sigma a double instead of a matrix?
		

		ostringstream k;
		k << "pose" << robotPoseNumber_;
		string posekey = k.str();
		k.clear();
		k << "lm" << landmarkNumber_;
		string lmkey = k.str();
		k.clear();

		return GaussianFactor::shared_ptr(
			new GaussianFactor(posekey, 
					   Dtransform_to1(sensorMatrix_, pose, landmark),
					   lmkey,
					   Dtransform_to2(sensorMatrix_, pose, landmark),
					   error_vector(config),
					   sigma_));
	}
} // namespace gtsam

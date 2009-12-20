/**
 * @file    UrbanMeasurement.cpp
 * @brief   A non-linear factor specialized to the Visual SLAM problem
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#include "UrbanMeasurement.h"
#include "Pose3.h"

using namespace std;

#define _GET_TEMPORARIES				\

namespace gtsam {

	/* ************************************************************************* */

	// JC:  I'd like to break this out into component calculations, but there's too much
	// Clearly there is a lot of code overlap (and redundant calculations) between 
	// this and the calculation of the relevant jacobians.  Need to look at cleaning
	// this up when I have a better grasp of the larger implications.  I don't think
	// there's any reason we can't just merge transform_to and the two derivative
	// functions into a single function to remove the overlap.
	



	UrbanMeasurement::Transform transform_to(const boost::shared_ptr<const Matrix> &sensorMatrix, 
						 const Pose3& robotpose, const Point2& lmpos,
						 bool getJacobians) {
		// Notation:
		// ph = phi = roll
		// th = theta = pitch
		// ps = psi = yaw
		Vector rpy = RQ(robotpose.rotation().matrix());	
		double dx = lmpos.x() - robotpose.translation().x();	
		double dy = lmpos.y() - robotpose.translation().y();	
		double s23 = (*sensorMatrix)(2, 3);		
		double cps = cos(rpy[2]);			
		double sps = sin(rpy[2]);			
		double secph = 1/cos(rpy[0]);			
		double tph = tan(rpy[0]);			
		double sph = sin(rpy[0]);			
		double sth = sin(rpy[1]);			
		double cth = cos(rpy[1]);			
		double tth = tan(rpy[1]);			
		double secth = 1/cth;
		
		UrbanMeasurement::Transform ret;
		ret.get<0>().reset(new Point2(cth*dy*sps
					    + (-s23*secph+dy*sps*sth+dx*sps*tph)*tth
					    + cps*(cth*dx+dx*sth*tth-dy*tph*tth),
					    secph*(cps*dy+s23*sph-dx*sps)));
		

		if (getJacobians) {
			ret.get<1>().reset(new Matrix(2, 6));
			Matrix &D1 = *(ret.get<1>());
			D1(0, 0) = (-cps*secth-sps*tph*tth);
			D1(0, 1) = (-secth*sps+cps*tph*tth);
			D1(0, 2) = 0;
			D1(0, 3) = -((secph*secph*(cps*dy+s23*sph-dx*sps)*tth));
			D1(0, 4) = (-((secth*(secth*(s23*secph+(cps*dy-dx*sps)*tph)
					       +(-cps*dx-dy*sps)*tth))));
			D1(0, 5) = ((cth*(cps*dy-dx*sps)+(cps*(dy*sth+dx*tph)
							   +sps*(-dx*sth+dy*tph))*tth));
			
			D1(1, 0) = secph*sps;
			D1(1, 1) = -cps*secph;
			D1(1, 2) = 0;
			D1(1, 3) = secph*(s23*secph+(cps*dy-dx*sps)*tph);
			D1(1, 4) = 0;
			D1(1, 5) = secph*(-cps*dx-dy*sps);

			
			ret.get<2>().reset(new Matrix(2, 2));
			Matrix &D2 = *(ret.get<2>());
			D2(0, 0) = cps*secth+sps*tph*tth;
			D2(0, 1) = secth*sps-cps*tph*tth;
			D2(1, 0) = -secph*sps;
			D2(1, 1) = cps*secph;
		}
		return ret;
	}

	
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
//		fprintf(stderr, "p: %.16g   l: %.16g\n",
//			pose.rotation().vector()[0],
//			landmark.x());
		boost::shared_ptr<Point2> h = transform_to(sensorMatrix_, pose,landmark, false).get<0>();
//		fprintf(stderr, "H: %.16g %.16g\nz: %.16g %.16g\n",
//			h->vector()[0], h->vector()[1],
//			z_[0], z_[1]);
		return (z_ - h->vector()); 
	}
	
	/* ************************************************************************* */
	GaussianFactor::shared_ptr UrbanMeasurement::linearize(const UrbanConfig& config) const {
		Pose3 pose = config.robotPose(robotPoseNumber_);
		Point2 landmark = config.landmarkPoint(landmarkNumber_);

		// JC: Things I don't yet understand:
		//
		// What should the keys be?  Presumably I just need to match a convention.
		//
		// Should the jacobians be premultiplied at the time of
		// GaussianFactor construction?
		//		
		// Is GaussianFactor copying its constructor arguments?  (If not, this is
		// not safe, nor is what this code replaced).
		//
		// Why is sigma a double instead of a matrix?
		

		ostringstream k;
		k << "pose" << robotPoseNumber_;
		string posekey = k.str();
		k.clear();
		k << "lm" << landmarkNumber_;
		string lmkey = k.str();
		k.clear();

		Transform tr = transform_to(sensorMatrix_, pose,landmark, true);
		

		return GaussianFactor::shared_ptr(
			new GaussianFactor(posekey, 
					   *(tr.get<1>()),
					   lmkey,
					   *(tr.get<2>()),
					   z_ - tr.get<0>()->vector(),
					   sigma_));
	}
} // namespace gtsam

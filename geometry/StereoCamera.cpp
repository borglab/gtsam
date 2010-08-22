/**
 * @file    StereoCamera.h
 * @brief   A Stereo Camera based on two Simple Cameras
 * @author  Chris Beall
 */

#include <gtsam/geometry/StereoCamera.h>

using namespace std;
using namespace gtsam;

namespace gtsam{

/* ************************************************************************* */
StereoCamera::StereoCamera(const Pose3& leftCamPose, const Cal3_S2& K, double baseline) :
	leftCamPose_(leftCamPose), K_(K), baseline_(baseline) {
	Vector calibration = K_.vector();
	fx_ = calibration(0);
	fy_ = calibration(1);
	cx_ = calibration(3);
	cy_ = calibration(4);
}

///* ************************************************************************* */
//StereoPoint2 StereoCamera::project(const Point3& point) const {
//
//	Point3 cameraPoint = leftCamPose_.transform_to(point);
//
//	double d = 1.0 / cameraPoint.z();
//	double uL = cx_ + d * fx_ *  cameraPoint.x();
//	double uR = cx_ + d * fx_ * (cameraPoint.x() - baseline_);
//	double v  = cy_ + d * fy_ *  cameraPoint.y();
//	return StereoPoint2(uL,uR,v);
//}

/* ************************************************************************* */
StereoPoint2 StereoCamera::project(const Point3& point,
		boost::optional<Matrix&> Dproject_stereo_pose,
		boost::optional<Matrix&> Dproject_stereo_point) const {

	Point3 cameraPoint = leftCamPose_.transform_to(point);

	if (Dproject_stereo_pose) {
		//Point2 intrinsic = project(camera.calibrated_, point);  // unused

		//Matrix D_intrinsic_pose = Dproject_pose(camera.calibrated_, point);
		//**** above function call inlined
		  Matrix D_cameraPoint_pose;
		  Point3 cameraPoint = pose().transform_to(point, D_cameraPoint_pose, boost::none);
		  //cout << "D_cameraPoint_pose" << endl;
		  //print(D_cameraPoint_pose);

		  //Point2 intrinsic = project_to_camera(cameraPoint);  // unused
		  Matrix D_intrinsic_cameraPoint = Dproject_to_stereo_camera1(cameraPoint); // 3x3 Jacobian
		  //cout << "myJacobian" << endl;
		  //print(D_intrinsic_cameraPoint);

		  Matrix D_intrinsic_pose = D_intrinsic_cameraPoint * D_cameraPoint_pose;

		//****
		//Matrix D_projection_intrinsic = Duncalibrate2(camera.K_, intrinsic);
		Matrix D_projection_intrinsic = Duncalibrate2(K());  // 3x3

		*Dproject_stereo_pose =  D_projection_intrinsic * D_intrinsic_pose;
	}

	if (Dproject_stereo_point) {
		//Point2 intrinsic = project(camera.calibrated_, point);   // unused

		//Matrix D_intrinsic_point = Dproject_point(camera.calibrated_, point);
		//**** above function call inlined
		  Matrix D_cameraPoint_point;
		  Point3 cameraPoint = pose().transform_to(point, boost::none, D_cameraPoint_point);

			//Point2 intrinsic = project_to_camera(cameraPoint);  // unused
			Matrix D_intrinsic_cameraPoint = Dproject_to_stereo_camera1(cameraPoint); // 3x3 Jacobian

			Matrix D_intrinsic_point = D_intrinsic_cameraPoint * D_cameraPoint_point;

		//****
		//Matrix D_projection_intrinsic = Duncalibrate2(camera.K_, intrinsic);
		Matrix D_projection_intrinsic = Duncalibrate2(K());  // 3x3

		*Dproject_stereo_point = D_projection_intrinsic * D_intrinsic_point;
	}


	double d = 1.0 / cameraPoint.z();
	double uL = cx_ + d * fx_ *  cameraPoint.x();
	double uR = cx_ + d * fx_ * (cameraPoint.x() - baseline_);
	double v  = cy_ + d * fy_ *  cameraPoint.y();
	return StereoPoint2(uL,uR,v);
}

/* ************************************************************************* */
Matrix StereoCamera::Dproject_to_stereo_camera1(const Point3& P) const {
	double d = 1.0 / P.z(), d2 = d * d;
	//return Matrix_(2, 3, d, 0.0, -P.x() * d2, 0.0, d, -P.y() * d2);
	return Matrix_(3, 3, d, 0.0, -P.x() * d2, d, 0.0, -(P.x()-baseline()) * d2, 0.0, d, -P.y() * d2);
}

/* ************************************************************************* */
Matrix StereoCamera::Duncalibrate2(const Cal3_S2& K) {
	Vector calibration = K.vector();  // I want fx, fx, fy
	Vector calibration2(3);
	calibration2(0) = calibration(0);
	calibration2(1) = calibration(0);
	calibration2(2) = calibration(1);
	return diag(calibration2);
	//return Matrix_(2, 2, K.fx_, K.s_, 0.000, K.fy_);
}

///* ************************************************************************* */
//Matrix Dproject_stereo_pose(const StereoCamera& camera, const Point3& point) {
//	//Point2 intrinsic = project(camera.calibrated_, point);  // unused
//
//	//Matrix D_intrinsic_pose = Dproject_pose(camera.calibrated_, point);
//	//**** above function call inlined
//	  Matrix D_cameraPoint_pose;
//	  Point3 cameraPoint = camera.pose().transform_to(point, D_cameraPoint_pose, boost::none);
//	  //cout << "D_cameraPoint_pose" << endl;
//	  //print(D_cameraPoint_pose);
//
//	  //Point2 intrinsic = project_to_camera(cameraPoint);  // unused
//	  Matrix D_intrinsic_cameraPoint = Dproject_to_stereo_camera1(camera, cameraPoint); // 3x3 Jacobian
//	  //cout << "myJacobian" << endl;
//	  //print(D_intrinsic_cameraPoint);
//
//	  Matrix D_intrinsic_pose = D_intrinsic_cameraPoint * D_cameraPoint_pose;
//
//	//****
//	//Matrix D_projection_intrinsic = Duncalibrate2(camera.K_, intrinsic);
//	Matrix D_projection_intrinsic = Duncalibrate2(camera.K());  // 3x3
//
//	Matrix D_projection_pose = D_projection_intrinsic * D_intrinsic_pose;
//	return D_projection_pose;
//}

///* ************************************************************************* */
//Matrix Dproject_stereo_point(const StereoCamera& camera, const Point3& point) {
//	//Point2 intrinsic = project(camera.calibrated_, point);   // unused
//
//	//Matrix D_intrinsic_point = Dproject_point(camera.calibrated_, point);
//	//**** above function call inlined
//	  Matrix D_cameraPoint_point;
//	  Point3 cameraPoint = camera.pose().transform_to(point, boost::none, D_cameraPoint_point);
//
//		//Point2 intrinsic = project_to_camera(cameraPoint);  // unused
//		Matrix D_intrinsic_cameraPoint = Dproject_to_stereo_camera1(camera, cameraPoint); // 3x3 Jacobian
//
//		Matrix D_intrinsic_point = D_intrinsic_cameraPoint * D_cameraPoint_point;
//
//	//****
//	//Matrix D_projection_intrinsic = Duncalibrate2(camera.K_, intrinsic);
//	Matrix D_projection_intrinsic = Duncalibrate2(camera.K());  // 3x3
//
//	Matrix D_projection_point = D_projection_intrinsic * D_intrinsic_point;
//	return D_projection_point;
//}


// calibrated cameras
/*
Matrix Dproject_pose(const CalibratedCamera& camera, const Point3& point) {
		Point3 cameraPoint = transform_to(camera.pose(), point);
		Matrix D_cameraPoint_pose = Dtransform_to1(camera.pose(), point);

		Point2 intrinsic = project_to_camera(cameraPoint);
		Matrix D_intrinsic_cameraPoint = Dproject_to_camera1(cameraPoint);

		Matrix D_intrinsic_pose = D_intrinsic_cameraPoint * D_cameraPoint_pose;
		return D_intrinsic_pose;
	}

Matrix Dproject_point(const CalibratedCamera& camera, const Point3& point) {
		Point3 cameraPoint = transform_to(camera.pose(), point);
		Matrix D_cameraPoint_point = Dtransform_to2(camera.pose());

		Point2 intrinsic = project_to_camera(cameraPoint);
		Matrix D_intrinsic_cameraPoint = Dproject_to_camera1(cameraPoint);

		Matrix D_intrinsic_point = D_intrinsic_cameraPoint * D_cameraPoint_point;
		return D_intrinsic_point;
	}
	*/

}

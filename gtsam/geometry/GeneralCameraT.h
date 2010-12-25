/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * DistortedCameraT.h
 *
 *  Created on: Mar 1, 2010
 *      Author: ydjian
 */

#pragma once
#ifndef GENERALCAMERAT_H_
#define GENERALCAMERAT_H_

#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/serialization/nvp.hpp>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/Cal3_S2.h>


#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>

namespace gtsam {

template <typename Camera, typename Calibration>
class GeneralCameraT {

	private:
		Camera calibrated_; // Calibrated camera
		Calibration calibration_; // Calibration

	public:
		GeneralCameraT(){}
		GeneralCameraT(const Camera& calibrated, const Calibration& K) : calibrated_(calibrated), calibration_(K) {}
		GeneralCameraT(const Camera& calibrated ) : calibrated_(calibrated) {}
		GeneralCameraT(const Pose3& pose, const Calibration& K) : calibrated_(pose), calibration_(K) {}
		GeneralCameraT(const Pose3& pose) :	calibrated_(pose) {}
		GeneralCameraT(const Pose3& pose, const Vector &v) : calibrated_(pose), calibration_(v) {}

		// Vector Initialization
		GeneralCameraT(const Vector &v) :
			calibrated_(subrange(v, 0, Camera::Dim())),
			calibration_(subrange(v, Camera::Dim(), Camera::Dim() + Calibration::Dim() )) {}

		inline const Pose3& pose() const { return calibrated_.pose(); }
		inline const Camera& calibrated() const { return calibrated_;	}
		inline const Calibration& calibration() const { return calibration_; }

		std::pair<Point2,bool> projectSafe(
				const Point3& P,
				boost::optional<Matrix&> H1,
				boost::optional<Matrix&> H2) const {

			Point3 cameraPoint = calibrated_.pose().transform_to(P);
			return std::pair<Point2, bool>(
					project(P,H1,H2),
					cameraPoint.z() > 0);
		}

		Point3 backproject(const Point2& projection, const double scale) const {
			Point2 intrinsic = calibration_.calibrate(projection);
			Point3 cameraPoint = CalibratedCamera::backproject_from_camera(intrinsic, scale);
			return calibrated_.pose().transform_from(cameraPoint);
		}

		/**
		 * project function that does not merge the Jacobians of calibration and pose
		 */
		Point2 project(const Point3& P, Matrix& H1_pose, Matrix& H1_k, Matrix& H2_pt) const {
			Matrix tmp;
			Point2 intrinsic = calibrated_.project(P, H1_pose, H2_pt);
			Point2 projection = calibration_.uncalibrate(intrinsic, H1_k, tmp);
			H1_pose = tmp * H1_pose;
			H2_pt = tmp * H2_pt;
			return projection;
		}

		/**
		 * project a 3d point to the camera
		 * P is point in camera coordinate
		 * H1 is respect to pose + calibration
		 * H2 is respect to landmark
		 */
		Point2 project(const Point3& P,
				boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const {

			if (!H1 && !H2) {
				Point2 intrinsic = calibrated_.project(P);
				return calibration_.uncalibrate(intrinsic) ;
			}

			Matrix H1_k, H1_pose, H2_pt;
			Point2 projection = project(P, H1_k, H1_pose, H2_pt);
			if ( H1 ) *H1 = collect(2, &H1_pose, &H1_k);
			if ( H2 ) *H2 = H2_pt;

			return projection;
		}

		// dump functions for compilation for now
		bool equals (const GeneralCameraT &camera, double tol = 1e-9) const {
			return calibrated_.equals(camera.calibrated_, tol) &&
				   calibration_.equals(camera.calibration_, tol) ;
		}

		void print(const std::string& s = "") const {
			calibrated_.pose().print(s + ".camera.") ;
			calibration_.print(s + ".calibration.") ;
		}

		GeneralCameraT expmap(const Vector &v) const {
			return GeneralCameraT(
					calibrated_.expmap(subrange(v,0,Camera::Dim())),
					calibration_.expmap(subrange(v,Camera::Dim(),Camera::Dim()+Calibration::Dim())));
		}

		Vector logmap(const GeneralCameraT &C) const {
			//std::cout << "logmap" << std::endl;
			const Vector v1(calibrated().logmap(C.calibrated())),
						 v2(calibration().logmap(C.calibration()));
			return concatVectors(2,&v1,&v2) ;
		}

		static GeneralCameraT Expmap(const Vector& v) {
			//std::cout << "Expmap" << std::endl;
			return GeneralCameraT(
				   Camera::Expmap(subrange(v,0,Camera::Dim())),
				   Calibration::Expmap(subrange(v,Camera::Dim(), Camera::Dim()+Calibration::Dim()))
				   );
		}

	    static Vector Logmap(const GeneralCameraT& p) {
	    	//std::cout << "Logmap" << std::endl;
	    	const Vector v1(Camera::Logmap(p.calibrated())),
	    			     v2(Calibration::Logmap(p.calibration()));
	    	return concatVectors(2,&v1,&v2);

	    }

		inline GeneralCameraT compose(const Pose3 &p) const {
			return GeneralCameraT( pose().compose(p), calibration_ ) ;
		}

		Matrix D2d_camera(const Point3& point) const {
			Point2 intrinsic = calibrated_.project(point);
			Matrix D_intrinsic_pose = Dproject_pose(calibrated_, point);
			Matrix D_2d_intrinsic = calibration_.D2d_intrinsic(intrinsic);
			Matrix D_2d_pose = prod(D_2d_intrinsic,D_intrinsic_pose);
			Matrix D_2d_calibration = calibration_.D2d_calibration(intrinsic);

			const int n1 = calibrated_.dim() ;
			const int n2 = calibration_.dim() ;
			Matrix D(2,n1+n2) ;

			subrange(D,0,2,0,n1) = D_2d_pose ;
			subrange(D,0,2,n1,n1+n2) = D_2d_calibration ;
			return D;
		}

		Matrix D2d_3d(const Point3& point) const {
			Point2 intrinsic = calibrated_.project(point);
			Matrix D_intrinsic_3d = Dproject_point(calibrated_, point);
			Matrix D_2d_intrinsic = calibration_.D2d_intrinsic(intrinsic);
			return prod(D_2d_intrinsic,D_intrinsic_3d);
		}

		Matrix D2d_camera_3d(const Point3& point) const {
			Point2 intrinsic = calibrated_.project(point);
			Matrix D_intrinsic_pose = Dproject_pose(calibrated_, point);
			Matrix D_2d_intrinsic = calibration_.D2d_intrinsic(intrinsic);
			Matrix D_2d_pose = prod(D_2d_intrinsic,D_intrinsic_pose);
			Matrix D_2d_calibration = calibration_.D2d_calibration(intrinsic);

			Matrix D_intrinsic_3d = Dproject_point(calibrated_, point);
			Matrix D_2d_3d = prod(D_2d_intrinsic,D_intrinsic_3d);

			const int n1 = calibrated_.dim() ;
			const int n2 = calibration_.dim() ;

			Matrix D(2,n1+n2+3) ;

			subrange(D,0,2,0,n1) = D_2d_pose ;
			subrange(D,0,2,n1,n1+n2) = D_2d_calibration ;
			subrange(D,0,2,n1+n2,n1+n2+3) = D_2d_3d ;
			return D;
		}

		//inline size_t dim() { return Camera::dim() + Calibration::dim() ; }
		inline size_t dim() const { return calibrated().dim() + calibration().dim() ; }
		static inline size_t Dim() { return Camera::Dim() + Calibration::Dim() ; }

private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & BOOST_SERIALIZATION_NVP(calibrated_);
			ar & BOOST_SERIALIZATION_NVP(calibration_);
		}

	};

typedef GeneralCameraT<CalibratedCamera,Cal3Bundler> Cal3BundlerCamera;
typedef GeneralCameraT<CalibratedCamera,Cal3DS2> Cal3DS2Camera;
typedef GeneralCameraT<CalibratedCamera,Cal3_S2> Cal3_S2Camera;

}

#endif

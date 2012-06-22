/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VisualSLAMData.cpp
 * @brief   Generate ground-truth simulated data for VisualSLAM examples
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <vector>
#include <map>
#include <gtsam/geometry/SimpleCamera.h>

/* ************************************************************************* */
/**
 * Simulated data for the visual SLAM examples:
 * - 8 Landmarks:	(10,10,10) (-10,10,10) (-10,-10,10) (10,-10,10)
 *      					(10,10,-10) (-10,10,-10) (-10,-10,-10) (10,-10,-10)
 * - n 90-deg-FoV cameras with the same calibration parameters:
 * 				 f = 50.0, Image: 100x100, center: 50.0, 50.0
 * 		and ground-truth poses on a circle around the landmarks looking at the world's origin:
 * 					Rot3(-sin(theta),  0, -cos(theta),
 * 							  cos(theta),  0, -sin(theta),
 * 							 					 0, -1,					 0 ),
 * 					Point3(r*cos(theta), r*sin(theta), 0.0)
 * 		(theta += 2*pi/N)
 * - Measurement noise: 1 pix sigma
 */
struct VisualSLAMExampleData {
	gtsam::shared_ptrK sK;								// camera calibration parameters
	std::vector<gtsam::Pose3> poses;          // ground-truth camera poses
	gtsam::Pose3 odometry;								// ground-truth odometry between 2 consecutive poses (simulated data for iSAM)
	std::vector<gtsam::Point3> points;     // ground-truth landmarks
	std::map<int, std::vector<gtsam::Point2> > z;	// 2D measurements of landmarks in each camera frame
	gtsam::SharedDiagonal noiseZ; 			// measurement noise (noiseModel::Isotropic::Sigma(2, 5.0f));
	gtsam::SharedDiagonal noiseX; 			// noise for camera poses
	gtsam::SharedDiagonal noiseL; 			// noise for landmarks

	static const VisualSLAMExampleData generate() {
		VisualSLAMExampleData data;
		// Landmarks (ground truth)
		data.points.push_back(gtsam::Point3(10.0,10.0,10.0));
		data.points.push_back(gtsam::Point3(-10.0,10.0,10.0));
		data.points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
		data.points.push_back(gtsam::Point3(10.0,-10.0,10.0));
		data.points.push_back(gtsam::Point3(10.0,10.0,-10.0));
		data.points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
		data.points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
		data.points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

		// Camera calibration parameters
		data.sK = gtsam::shared_ptrK(new gtsam::Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

		// n camera poses
		int n = 8;
		double theta = 0.0;
		double r = 30.0;
		for (int i=0; i<n; ++i, theta += 2*M_PI/n) {
			gtsam::Point3 C = gtsam::Point3(r*cos(theta), r*sin(theta), 0.0);
			gtsam::SimpleCamera camera = gtsam::SimpleCamera::lookat(C, gtsam::Point3(), gtsam::Point3(0,0,1));
			data.poses.push_back(camera.pose());
		}
		data.odometry = data.poses[0].between(data.poses[1]);

		// Simulated measurements, possibly with Gaussian noise
		data.noiseZ = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
		for (size_t i=0; i<data.poses.size(); ++i) {
			for (size_t j=0; j<data.points.size(); ++j) {
				gtsam::SimpleCamera camera(data.poses[i], *data.sK);
				data.z[i].push_back(camera.project(data.points[j])
						/*+ gtsam::Point2(data.noiseZ->sample()))*/); // you can add noise as desired
			}
		}
		data.noiseX = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(6, 0.001, 0.001, 0.001, 0.1, 0.1, 0.1));
		data.noiseL = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

		return data;
	}
};

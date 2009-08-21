/**
* @file   Simulated3D.h
* @brief  measurement functions and derivatives for simulated 3D robot
* @author Alex Cunningham
**/

// \callgraph

#pragma once

#include "NonlinearFactor.h"

// \namespace

namespace gtsam {
	
	/**
	* Prior on a single pose
	*/
	Vector prior_3D (const Vector& x); 
	Matrix Dprior_3D(const Vector& x);
	
	/**
	* odometry between two poses
	*/
	Vector odo_3D(const Vector& x1, const Vector& x2);
	Matrix Dodo1_3D(const Vector& x1, const Vector& x2);
	Matrix Dodo2_3D(const Vector& x1, const Vector& x2);
	
	/**
	*  measurement between landmark and pose
	*/
	Vector mea_3D(const Vector& x,  const Vector& l);
	Matrix Dmea1_3D(const Vector& x, const Vector& l);
	Matrix Dmea2_3D(const Vector& x, const Vector& l);
	
	struct Point2Prior3D : public NonlinearFactor1 {
		Point2Prior3D(const Vector& mu, double sigma, const std::string& key):
		NonlinearFactor1(mu, sigma, prior_3D, key, Dprior_3D) {}
	};
	
	struct Simulated3DMeasurement : public NonlinearFactor2 {
		Simulated3DMeasurement(const Vector& z, double sigma, const std::string& key1, const std::string& key2):
		NonlinearFactor2(z, sigma, mea_3D, key1, Dmea1_3D, key2, Dmea2_3D) {}
	};
	
} // namespace gtsam
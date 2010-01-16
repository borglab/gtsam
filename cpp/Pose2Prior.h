/**
 *  @file  Pose2Prior.h
 *  @authors Michael Kaess
 **/
#pragma once

#include <map>
#include "NonlinearFactor.h"
#include "GaussianFactor.h"
#include "Pose2.h"
#include "Matrix.h"
#include "Key.h"
#include "Pose2Graph.h"
#include "ostream"

namespace gtsam {

class Pose2Prior : public NonlinearFactor<Pose2Config> {
private:
	typedef Pose2Config::Key Key;
	Key key_;
	Pose2 measured_;
	Matrix square_root_inverse_covariance_; /** sqrt(inv(measurement_covariance)) */
	//std::list<Key> keys_;

public:

	typedef NonlinearFactor<Pose2Config> Base;
	typedef boost::shared_ptr<Pose2Prior> shared_ptr; // shorthand for a smart pointer to a factor

	Pose2Prior(const Key& key, const Pose2& measured, const Matrix& measurement_covariance) :
			  key_(key),measured_(measured),Base(1.0) {
		square_root_inverse_covariance_ = inverse_square_root(measurement_covariance);
		keys_.push_back(key);
	}

	/** implement functions needed for Testable */
	void print(const std::string& name) const {
		std::cout << name << std::endl;
		std::cout << "Factor "<< std::endl;
		std::cout << "key "<< (std::string)key_<<std::endl;
		measured_.print("measured ");
		gtsam::print(square_root_inverse_covariance_,"MeasurementCovariance");
	}
	bool equals(const NonlinearFactor<Pose2Config>& expected, double tol) const {return false;}

	/** implement functions needed to derive from Factor */
	Vector error_vector(const Pose2Config& config) const {
		Pose2 p = config[key_];
		return square_root_inverse_covariance_ * logmap(measured_,p);
	}

	//std::list<Key> keys() const { return keys_; }
	std::size_t size() const { return keys_.size(); }

	/** linearize */
	boost::shared_ptr<GaussianFactor> linearize(const Pose2Config& config) const {
		Pose2 p = config[key_];
		Vector b = - error_vector(config);
		Matrix H(3,3);
		H(0,0)=1; H(0,1)=0; H(0,2)=0;
		H(1,0)=0; H(1,1)=1; H(1,2)=0;
		H(2,0)=0; H(2,1)=0; H(2,2)=1;
		boost::shared_ptr<GaussianFactor> linearized(new GaussianFactor(
				key_, square_root_inverse_covariance_ * H,
				b, 1.0));
		return linearized;
	}
};
} /// namespace gtsam

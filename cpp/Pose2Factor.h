/**
 *  @file  Pose2Factor.H
 *  @authors Frank Dellaert, Viorela Ila
 **/
#pragma once

#include <map>
#include "NonlinearFactor.h"
#include "GaussianFactor.h"
#include "VectorConfig.h"
#include "Pose2.h"
#include "Matrix.h"
#include "Pose2Config.h"
#include "ostream"

namespace gtsam {

class Pose2Factor : public NonlinearFactor<Pose2Config> {
private:
	std::string key1_, key2_; /** The keys of the two poses, order matters */
	Pose2 measured_;
	Matrix square_root_inverse_covariance_; /** sqrt(inv(measurement_covariance)) */
	std::list<std::string> keys_;

public:

	typedef boost::shared_ptr<Pose2Factor> shared_ptr; // shorthand for a smart pointer to a factor

	Pose2Factor(const std::string& key1, const std::string& key2,
			const Pose2& measured, const Matrix& measurement_covariance) :
			  key1_(key1),key2_(key2),measured_(measured) {
		square_root_inverse_covariance_ = inverse_square_root(measurement_covariance);
		keys_.push_back(key1);
		keys_.push_back(key2);
	}

	/** implement functions needed for Testable */
	void print(const std::string& name) const {
		std::cout << name << std::endl;
		std::cout << "Factor "<< std::endl;
		std::cout << "key1 "<< key1_<<std::endl;
		std::cout << "key2 "<< key2_<<std::endl;
		measured_.print("measured ");
		gtsam::print(square_root_inverse_covariance_,"MeasurementCovariance");
	}
	bool equals(const NonlinearFactor<Pose2Config>& expected, double tol) const {return false;}

	/** implement functions needed to derive from Factor */
	Vector error_vector(const Pose2Config& config) const {
		//z-h
		Pose2 p1 = config.get(key1_), p2 = config.get(key2_);
		return -between(p1,p2).log(measured_);
	}

	std::list<std::string> keys() const { return keys_; }
	std::size_t size() const { return keys_.size(); }

	/** linearize */
	boost::shared_ptr<GaussianFactor> linearize(const Pose2Config& config) const {
		Pose2 p1 = config.get(key1_), p2 = config.get(key2_);
		Vector b = -between(p1,p2).log(measured_);
		Matrix H1 = Dbetween1(p1,p2);
		Matrix H2 = Dbetween2(p1,p2);
		boost::shared_ptr<GaussianFactor> linearized(new GaussianFactor(
				key1_, square_root_inverse_covariance_ * H1,
				key2_, square_root_inverse_covariance_ * H2,
				b, 1.0));
		return linearized;
	}
};
} /// namespace gtsam

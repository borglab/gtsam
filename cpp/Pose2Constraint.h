/**
 *  @file  Pose2Constraint.H
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <map>
#include "GaussianFactor.h"
#include "VectorConfig.h"
#include "Pose2.h"
#include "Matrix.h"

namespace gtsam {

class Pose2Config : public std::map<std::string,Pose2> {
public:
	Pose2 get(std::string key) const {
		std::map<std::string,Pose2>::const_iterator it = find(key);
		if (it==end()) throw std::invalid_argument("invalid key");
		return it->second;
	}
};

class Pose2Constraint : public Factor<Pose2Config> {
private:
	std::string key1_, key2_; /** The keys of the two poses, order matters */
	Pose2 measured_;
	Matrix square_root_inverse_covariance_; /** sqrt(inv(measurement_covariance)) */

public:
	Pose2Constraint(const std::string& key1, const std::string& key2,
			const Pose2& measured, const Matrix& measurement_covariance): key1_(key1),key2_(key2),measured_(measured) {
		square_root_inverse_covariance_ = inverse_square_root(measurement_covariance);
	}

	/** implement functions needed for Testable */
	void print(const std::string& name) const {}
	bool equals(const Factor<Pose2Config>& expected, double tol) const {return false;}

	/** implement functions needed to derive from Factor */
	double error(const Pose2Config& c) const {return 0;}
	std::list<std::string> keys() const { std::list<std::string> l; return l; }
	std::size_t size() const { return 2;}

	/** linearize */
	boost::shared_ptr<GaussianFactor> linearize(const Pose2Config& config) const {
		Pose2 p1 = config.get(key1_), p2 = config.get(key2_);
		Vector b = (measured_ - between(p1,p2)).vector();
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

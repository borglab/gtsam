/**
 * @file    VectorConfig.cpp
 * @brief   Pose2Graph Configuration
 * @author  Frank Dellaert
 */

#include <boost/foreach.hpp>
#include "VectorConfig.h"
#include "Pose2Config.h"

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	const Pose2& Pose2Config::get(std::string key) const {
		std::map<std::string, Pose2>::const_iterator it = values_.find(key);
		if (it == values_.end()) throw std::invalid_argument("invalid key");
		return it->second;
	}

	/* ************************************************************************* */
	void Pose2Config::insert(const std::string& name, const Pose2& val) {
		values_.insert(make_pair(name, val));
	}

	/* ************************************************************************* */
	bool Pose2Config::equals(const Pose2Config& expected, double tol) const {
		if (values_.size() != expected.values_.size()) return false;

		// iterate over all nodes
		string j;
		Pose2 pj;
		FOREACH_PAIR(j, pj, values_)
			if(!pj.equals(expected.get(j),tol))
				return false;
		return true;
	}

	/* ************************************************************************* */
	void Pose2Config::print(const std::string &s) const {
		std::cout << "Pose2Config " << s << ", size " << values_.size() << "\n";
		std::string j; Pose2 pj;
		FOREACH_PAIR(j, pj, values_)
			pj.print(j + ": ");
	}

	/* ************************************************************************* */
	Pose2Config expmap(const Pose2Config& c, const VectorConfig& delta) {
		Pose2Config newConfig;
		std::string j; Pose2 pj;
		FOREACH_PAIR(j, pj, c.values_) {
			if (delta.contains(j)) {
				const Vector& dj = delta[j];
				//check_size(j,vj,dj);
				newConfig.insert(j, expmap(pj,dj));
			} else
				newConfig.insert(j, pj);
		}
		return newConfig;
	}

	/* ************************************************************************* */
	// TODO: local version, should probably defined in LieConfig
	static string symbol(char c, int index) {
		stringstream ss;
		ss << c << index;
		return ss.str();
	}

	/* ************************************************************************* */
	Pose2Config pose2Circle(size_t n, double R, char c) {
		Pose2Config x;
		double theta = 0, dtheta = 2*M_PI/n;
		for(size_t i=0;i<n;i++, theta+=dtheta)
			x.insert(symbol(c,i), Pose2(cos(theta), sin(theta), M_PI_2 + theta));
		return x;
	}

	/* ************************************************************************* */
} // namespace

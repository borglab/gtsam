#pragma once

#include <map>
#include <boost/exception.hpp>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <iostream>
#include <string>
#include "Pose2.h"
#include "Testable.h"
#include "VectorConfig.h"
#include "Vector.h"

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)


namespace gtsam {

class Pose2Config: public Testable<Pose2Config> {

private:
  std::map<std::string, Pose2> values;

public:

	Pose2Config() {}
	Pose2Config(const Pose2Config &config) : values(config.values) { }
	virtual ~Pose2Config() { }

	Pose2 get(std::string key) const {
		std::map<std::string, Pose2>::const_iterator it = values.find(key);
		if (it == values.end())
			throw std::invalid_argument("invalid key");
		return it->second;
	}
	void insert(const std::string& name, const Pose2& val){
		values.insert(make_pair(name, val));
	}

	Pose2Config& operator=(Pose2Config& rhs) {
	  values = rhs.values;
	  return (*this);
	}

	bool equals(const Pose2Config& expected, double tol) const {
	  std::cerr << "Pose2Config::equals not implemented!" << std::endl;
	  throw "Pose2Config::equals not implemented!";
	}

	void print(const std::string &s) const {
	  std::cout << s << std::endl;
	}

    /**
     * Add a delta config, needed for use in NonlinearOptimizer
     */
    Pose2Config exmap(const VectorConfig& delta) const {
      Pose2Config newConfig;
      std::string j; Pose2 vj;
      FOREACH_PAIR(j, vj, values) {
          if (delta.contains(j)) {
              const Vector& dj = delta[j];
              //check_size(j,vj,dj);
              newConfig.insert(j, vj.exmap(dj));
          } else {
              newConfig.insert(j, vj);
          }
      }
      return newConfig;
    }

};
} // namespace

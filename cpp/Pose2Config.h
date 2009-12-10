#include <map>
#include "Pose2.h"

namespace gtsam {

class Pose2Config: public std::map<std::string, Pose2> {

public:

	Pose2Config() {}

	Pose2 get(std::string key) const {
		std::map<std::string, Pose2>::const_iterator it = find(key);
		if (it == end())
			throw std::invalid_argument("invalid key");
		return it->second;
	}
};
} // namespace

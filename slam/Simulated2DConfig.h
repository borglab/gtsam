/*
 * Simulated2DConfig.h
 *
 * Re-created on Feb 22, 2010 for compatibility with MATLAB
 * Author: Frank Dellaert
 */

#pragma once

#include <gtsam/slam/simulated2D.h>

namespace gtsam {

	class Simulated2DConfig: public simulated2D::Config {
	public:
		typedef boost::shared_ptr<Point2> sharedPoint;

		Simulated2DConfig() {
		}

		void insertPose(const simulated2D::PoseKey& i, const Point2& p) {
			insert(i, p);
		}

		void insertPoint(const simulated2D::PointKey& j, const Point2& p) {
			insert(j, p);
		}

		int nrPoses() const {
			return this->first_.size();
		}

		int nrPoints() const {
			return this->second_.size();
		}

		sharedPoint pose(const simulated2D::PoseKey& i) {
			return sharedPoint(new Point2((*this)[i]));
		}

		sharedPoint point(const simulated2D::PointKey& j) {
			return sharedPoint(new Point2((*this)[j]));
		}

	};

} // namespace gtsam


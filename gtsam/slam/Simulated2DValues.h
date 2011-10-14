/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Simulated2DValues.h
 * @brief Re-created on Feb 22, 2010 for compatibility with MATLAB
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/simulated2D.h>

namespace gtsam {

	class Simulated2DValues: public simulated2D::Values {
	public:
		typedef boost::shared_ptr<Point2> sharedPoint;

		Simulated2DValues() {
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


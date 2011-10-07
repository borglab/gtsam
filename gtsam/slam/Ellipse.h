/**
 * @file Ellipse.h
 * @brief Data structure for managing covariance ellipses with visualization
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/geometry/Pose2.h>

namespace gtsam {

/** Parameters and functions for a 2D covariance ellipse */
class Ellipse {
protected:
	Point2 mean_;
	Matrix cov_;
	Point2 moment1_, moment2_; /// moments of the ellipse

	// TODO: make this static so it isn't calculated each time
	std::vector<Point2> unit_circle_; /// unit circle for ellipses

public:

	/** finds the moments on construction */
	Ellipse(const Point2& mean, const Matrix& covariance);
	Ellipse(const Pose2& mean, const Matrix& covariance);

	// access
	const Point2& mean() const { return mean_; }
	const Matrix& covariance() const { return cov_; }
	const Point2& moment1() const { return moment1_; }
	const Point2& moment2() const { return moment2_; }
	const std::vector<Point2>& unit_circle() const { return unit_circle_; }

	void print(const std::string& s="") const;

protected:
	/** calculates moments from the covariance matrix */
	void calculateMoments(const Matrix& covariance);
};

} // \namespace gtsam

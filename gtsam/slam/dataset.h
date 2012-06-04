/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file dataset.h
 * @date Jan 22, 2010
 * @author nikai
 * @brief utility functions for loading datasets
 */

#pragma once

#include <gtsam/slam/pose2SLAM.h>
#include <string>

namespace gtsam {
	/**
	 * Construct dataset filename from short name
	 * Currently has "Killian" "intel.gfs", "10K", etc...
	 * @param filename
	 * @param optional dataset, if empty will try to getenv $DATASET
	 * @param optional path, if empty will try to getenv $HOME
	 */
	std::pair<std::string, boost::optional<gtsam::SharedDiagonal> >
	dataset(const std::string& dataset = "", const std::string& path = "");

	/**
	 * Load TORO 2D Graph
	 * @param dataset/model pair as constructed by [dataset]
	 * @param maxID if non-zero cut out vertices >= maxID
	 * @param addNoise add noise to the edges
	 * @param smart try to reduce complexity of covariance to cheapest model
	 */
	std::pair<pose2SLAM::Graph::shared_ptr, pose2SLAM::Values::shared_ptr> load2D(
			std::pair<std::string, boost::optional<SharedDiagonal> > dataset,
			int maxID = 0, bool addNoise = false, bool smart = true);

	/**
	 * Load TORO 2D Graph
	 * @param filename
	 * @param model optional noise model to use instead of one specified by file
	 * @param maxID if non-zero cut out vertices >= maxID
	 * @param addNoise add noise to the edges
	 * @param smart try to reduce complexity of covariance to cheapest model
	 */
	std::pair<pose2SLAM::Graph::shared_ptr, pose2SLAM::Values::shared_ptr> load2D(
			const std::string& filename,
			boost::optional<gtsam::SharedDiagonal> model = boost::optional<
					gtsam::SharedDiagonal>(), int maxID = 0, bool addNoise = false,
			bool smart = true);

	/** save 2d graph */
	void save2D(const pose2SLAM::Graph& graph, const Values& config,
			const gtsam::SharedDiagonal model, const std::string& filename);

	/**
	 * Load TORO 3D Graph
	 */
	bool load3D(const std::string& filename);

} // namespace gtsam

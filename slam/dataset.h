/*
 * dataset.h
 *
 *   Created on: Jan 22, 2010
 *       Author: nikai
 *  Description: utility functions for loading datasets
 */

#pragma once


#include <string>
#include <boost/shared_ptr.hpp>
#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/inference/graph.h>

namespace gtsam
{
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
 * @param filename
 * @param maxID, if non-zero cut out vertices >= maxID
 * @param smart: try to reduce complexity of covariance to cheapest model
 */
std::pair<boost::shared_ptr<gtsam::Pose2Graph>, boost::shared_ptr<gtsam::Pose2Values> > load2D(
		std::pair<std::string, boost::optional<SharedDiagonal> > dataset,
		int maxID = 0, bool addNoise=false, bool smart=true);
std::pair<boost::shared_ptr<gtsam::Pose2Graph>, boost::shared_ptr<gtsam::Pose2Values> > load2D(
		const std::string& filename,
		boost::optional<gtsam::SharedDiagonal> model = boost::optional<gtsam::SharedDiagonal>(),
		int maxID = 0, bool addNoise=false, bool smart=true);

/** save 2d graph */
void save2D(const gtsam::Pose2Graph& graph, const gtsam::Pose2Values& config, const gtsam::SharedDiagonal model,
		const std::string& filename);

/**
 * Load TORO 3D Graph
 */
bool load3D(const std::string& filename);

} // namespace gtsam

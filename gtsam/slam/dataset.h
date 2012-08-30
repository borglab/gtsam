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

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>

#include <string>

namespace gtsam {

#ifndef MATLAB_MEX_FILE
	/**
	 * Find the full path to an example dataset distributed with gtsam.  The name
	 * may be specified with or without a file extension - if no extension is
	 * give, this function first looks for the .graph extension, then .txt.  We
	 * first check the gtsam source tree for the file, followed by the installed
	 * example dataset location.  Both the source tree and installed locations
	 * are obtained from CMake during compilation.
	 * @return The full path and filename to the requested dataset.
	 * @throw std::invalid_argument if no matching file could be found using the
	 * search process described above.
	 */
	std::string findExampleDataFile(const std::string& name);
#endif

	/**
	 * Load TORO 2D Graph
	 * @param dataset/model pair as constructed by [dataset]
	 * @param maxID if non-zero cut out vertices >= maxID
	 * @param addNoise add noise to the edges
	 * @param smart try to reduce complexity of covariance to cheapest model
	 */
	std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> load2D(
			std::pair<std::string, boost::optional<noiseModel::Diagonal::shared_ptr> > dataset,
			int maxID = 0, bool addNoise = false, bool smart = true);

	/**
	 * Load TORO 2D Graph
	 * @param filename
	 * @param model optional noise model to use instead of one specified by file
	 * @param maxID if non-zero cut out vertices >= maxID
	 * @param addNoise add noise to the edges
	 * @param smart try to reduce complexity of covariance to cheapest model
	 */
	std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> load2D(
			const std::string& filename,
			boost::optional<gtsam::SharedDiagonal> model = boost::optional<
					noiseModel::Diagonal::shared_ptr>(), int maxID = 0, bool addNoise = false,
			bool smart = true);

	/** save 2d graph */
	void save2D(const NonlinearFactorGraph& graph, const Values& config,
			const noiseModel::Diagonal::shared_ptr model, const std::string& filename);

	/**
	 * Load TORO 3D Graph
	 */
	bool load3D(const std::string& filename);

} // namespace gtsam

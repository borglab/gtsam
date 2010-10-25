/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * Pose2SLAMOptimizer.cpp
 *
 *  Created on: Jan 22, 2010
 *      Author: dellaert
 */

#include <gtsam/slam/Pose2SLAMOptimizer.h>
#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/linear/SubgraphSolver-inl.h>

using namespace std;
namespace gtsam {

	using namespace pose2SLAM;

	/* ************************************************************************* */
	Pose2SLAMOptimizer::Pose2SLAMOptimizer(const string& dataset_name,
			const string& path) {

		static int maxID = 0;
		static bool addNoise = false;

		string filename;
		boost::optional<SharedDiagonal> noiseModel;
		boost::tie(filename, noiseModel) = dataset(dataset_name);

		// read graph and initial estimate
		boost::tie(graph_, theta_) = load2D(filename, noiseModel, maxID, addNoise);
		graph_->addPrior(theta_->begin()->first, theta_->begin()->second,
				noiseModel::Unit::Create(3));

		// initialize non-linear solver
		solver_.initialize(*graph_, *theta_);

		linearize();
	}

	/* ************************************************************************* */
	void Pose2SLAMOptimizer::print(const string& str) const {
		GTSAM_PRINT(*graph_);
		GTSAM_PRINT(*theta_);
		//TODO
		//GTSAM_PRINT(solver_);
		GTSAM_PRINT(*system_);
	}

	/* ************************************************************************* */
	void Pose2SLAMOptimizer::update(const Vector& x) {
		VectorValues X = system_->assembleValues(x, *solver_.ordering());
		*theta_ = theta_->expmap(X);
		linearize();
	}

	/* ************************************************************************* */
	void Pose2SLAMOptimizer::updatePreconditioned(const Vector& y) {
		Vector x;
		update(x);
	}

	/* ************************************************************************* */
	Vector Pose2SLAMOptimizer::optimize() const {
		VectorValues X = solver_.optimize(*system_);
		std::list<Vector> vs;
		BOOST_FOREACH(const Symbol& key, *solver_.ordering())
			vs.push_back(X[key]);
		return concatVectors(vs);
	}

	/* ************************************************************************* */
	double Pose2SLAMOptimizer::error() const {
		return graph_->error(*theta_);
	}
}

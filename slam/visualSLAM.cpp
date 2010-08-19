/*
 * visualSLAM.cpp
 *
 *  Created on: Jan 14, 2010
 *      Author: richard
 */

#include <gtsam/slam/visualSLAM.h>
#include <gtsam/nonlinear/TupleConfig-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>

namespace gtsam {
	INSTANTIATE_TUPLE_CONFIG2(visualSLAM::PoseConfig, visualSLAM::PointConfig)
	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(visualSLAM::Config)
	INSTANTIATE_NONLINEAR_OPTIMIZER(visualSLAM::Graph, visualSLAM::Config)

}

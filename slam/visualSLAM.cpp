/*
 * visualSLAM.cpp
 *
 *  Created on: Jan 14, 2010
 *      Author: richard
 */

#include <gtsam/slam/visualSLAM.h>
#include <gtsam/nonlinear/TupleValues-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>

namespace gtsam {
	INSTANTIATE_TUPLE_CONFIG2(visualSLAM::PoseValues, visualSLAM::PointValues)
	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(visualSLAM::Values)
	INSTANTIATE_NONLINEAR_OPTIMIZER(visualSLAM::Graph, visualSLAM::Values)

}

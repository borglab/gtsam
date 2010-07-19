/*
 * visualSLAM.cpp
 *
 *  Created on: Jan 14, 2010
 *      Author: richard
 */

#include "visualSLAM.h"
#include "TupleConfig-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "NonlinearFactorGraph-inl.h"

namespace gtsam {
	INSTANTIATE_TUPLE_CONFIG2(visualSLAM::PoseConfig, visualSLAM::PointConfig)
	INSTANTIATE_NONLINEAR_FACTOR_GRAPH(visualSLAM::Config)
	INSTANTIATE_NONLINEAR_OPTIMIZER(visualSLAM::Graph, visualSLAM::Config)

}

/**
 * @file    VSLAMConfig.cpp
 * @brief   LieConfig instantiations
 * @author  Frank Dellaert
 */

#include "VSLAMConfig.h"
#include "LieConfig-inl.h"
#include "TupleConfig.h"

namespace gtsam {

	// template class LieConfig<VSLAMPoseKey, Pose3> ; // not this one as duplicate
	template class LieConfig<VSLAMPointKey, Point3> ;
	template class PairConfig<VSLAMPoseKey, Pose3, VSLAMPointKey, Point3> ;

/* ************************************************************************* */

} // namespace gtsam


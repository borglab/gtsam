/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file WnoaFactorGraphInstantiations.cpp
 * @brief Explicit instantiations of the WNOA graph for supported states.
 */

#include <gtsam/dllexport.h>
#include <gtsam/geometry/Point1.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/WnoaFactorGraph.h>
#include <gtsam/nonlinear/internal/WnoaFactorGraph-inl.h>  // IWYU pragma: keep

namespace gtsam {

template class GTSAM_EXPORT WnoaFactorGraph<Point1>;
template class GTSAM_EXPORT WnoaFactorGraph<Point2>;
template class GTSAM_EXPORT WnoaFactorGraph<Point3>;
template class GTSAM_EXPORT WnoaFactorGraph<Pose2>;
template class GTSAM_EXPORT WnoaFactorGraph<Pose3>;
}  // namespace gtsam

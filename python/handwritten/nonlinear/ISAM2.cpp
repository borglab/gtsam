/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief exports ISAM2 class to python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/geometry/Pose3.h"

using namespace boost::python;
using namespace gtsam;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(update_overloads, ISAM2::update, 0, 7)

void exportISAM2(){

// TODO(Ellon): Export all properties of ISAM2Params
class_<ISAM2Params>("ISAM2Params")
  .add_property("relinearize_skip", &ISAM2Params::getRelinearizeSkip, &ISAM2Params::setRelinearizeSkip)
  .add_property("enable_relinearization", &ISAM2Params::isEnableRelinearization, &ISAM2Params::setEnableRelinearization)
  .add_property("evaluate_non_linear_error", &ISAM2Params::isEvaluateNonlinearError, &ISAM2Params::setEvaluateNonlinearError)
  .add_property("factorization", &ISAM2Params::getFactorization, &ISAM2Params::setFactorization)
  .add_property("cache_linearized_factors", &ISAM2Params::isCacheLinearizedFactors, &ISAM2Params::setCacheLinearizedFactors)
  .add_property("enable_detailed_results", &ISAM2Params::isEnableDetailedResults, &ISAM2Params::setEnableDetailedResults)
  .add_property("enable_partial_linearization_check", &ISAM2Params::isEnablePartialRelinearizationCheck, &ISAM2Params::setEnablePartialRelinearizationCheck)
  // TODO(Ellon): Check if it works with FastMap; Implement properly if it doesn't.
  .add_property("relinearization_threshold", &ISAM2Params::getRelinearizeThreshold, &ISAM2Params::setRelinearizeThreshold)
  // TODO(Ellon): Wrap the following setters/getters:
  //     void 	setOptimizationParams (OptimizationParams optimizationParams)
  //     OptimizationParams 	getOptimizationParams () const
  //     void 	setKeyFormatter (KeyFormatter keyFormatter)
  //     KeyFormatter 	getKeyFormatter () const
  //     GaussianFactorGraph::Eliminate 	getEliminationFunction () const 
;

// TODO(Ellon): Export useful methods/properties of ISAM2Result
class_<ISAM2Result>("ISAM2Result")
;

// Function pointers for overloads in ISAM2
Values (ISAM2::*calculateEstimate_0)() const = &ISAM2::calculateEstimate;

class_<ISAM2>("ISAM2")
  .def(init<const ISAM2Params &>())
  // TODO(Ellon): wrap all optional values of update
  .def("update",&ISAM2::update, update_overloads())
  .def("calculate_estimate", calculateEstimate_0)
  .def("calculate_pose3_estimate", &ISAM2::calculateEstimate<Pose3>, (arg("self"), arg("key")) )
  .def("value_exists", &ISAM2::valueExists)
;

}
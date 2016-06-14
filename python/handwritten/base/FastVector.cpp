 /* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps FastVector instances to python
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/base/FastVector.h"
#include "gtsam/base/types.h" // for Key definition

using namespace boost::python;
using namespace gtsam;

void exportFastVectors(){

  typedef FastVector<Key> KeyVector;

  class_<KeyVector>("KeyVector")
  	.def(vector_indexing_suite<KeyVector>())
  ;

}
 /* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief register conversion matrix between numpy and Eigen
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/base/Matrix.h"
#include "gtsam/base/Vector.h"

using namespace boost::python;
using namespace gtsam;

void registerNumpyEigenConversions()
{
  // NOTE: import array should be called only in the cpp defining the module
  // import_array();
  NumpyEigenConverter<Vector>::register_converter();
  NumpyEigenConverter<Vector1>::register_converter();
  NumpyEigenConverter<Vector2>::register_converter();
  NumpyEigenConverter<Vector3>::register_converter();
  NumpyEigenConverter<Vector4>::register_converter();
  NumpyEigenConverter<Vector5>::register_converter();
  NumpyEigenConverter<Vector6>::register_converter();
  NumpyEigenConverter<Vector7>::register_converter();
  NumpyEigenConverter<Vector8>::register_converter();
  NumpyEigenConverter<Vector9>::register_converter();
  NumpyEigenConverter<Vector10>::register_converter();
  
  NumpyEigenConverter<Matrix>::register_converter();
  NumpyEigenConverter<Matrix2>::register_converter();
  NumpyEigenConverter<Matrix3>::register_converter();
  NumpyEigenConverter<Matrix4>::register_converter();
  NumpyEigenConverter<Matrix5>::register_converter();
  NumpyEigenConverter<Matrix6>::register_converter();
  NumpyEigenConverter<Matrix7>::register_converter();
  NumpyEigenConverter<Matrix8>::register_converter();
  NumpyEigenConverter<Matrix9>::register_converter();

}

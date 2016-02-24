/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief exports the python module
 * @author Andrew Melim
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#include <boost/python.hpp>
#include <numpy_eigen/NumpyEigenConverter.hpp>

// base
void exportFastVectors();

// geometry
void exportPoint2();
void exportPoint3();
void exportRot2();
void exportRot3();
void exportPose2();
void exportPose3();
void exportPinholeBaseK();
void exportPinholeCamera();
void exportCal3_S2();
void export_geometry();

// inference
void exportSymbol();

// linear
void exportNoiseModels();

// nonlinear
void exportValues();
void exportNonlinearFactor();
void exportNonlinearFactorGraph();
void exportLevenbergMarquardtOptimizer();
void exportISAM2();

// slam
void exportPriorFactors();
void exportBetweenFactors();
void exportGenericProjectionFactor();
void export_slam();

// navigation
void exportImuFactor();
void exportScenario();


// Utils (or Python wrapper specific functions)
void registerNumpyEigenConversions();

//-----------------------------------//

BOOST_PYTHON_MODULE(gtsampy){

  // NOTE: We need to call import_array1() instead of import_array() to support both python 2
  //       and 3. The reason is that BOOST_PYTHON_MODULE puts all its contents in a function
  //       returning void, and import_array() is a macro that when expanded for python 3, adds
  //       a 'return __null' statement to that function. For more info check files:
  //       boost/python/module_init.hpp and numpy/__multiarray_api.h (bottom of the file).
  // Should be the first thing to be done
  import_array1();

  registerNumpyEigenConversions();

  exportFastVectors();

  exportPoint2();
  exportPoint3();
  exportRot2();
  exportRot3();
  exportPose2();
  exportPose3();
  exportPinholeBaseK();
  exportPinholeCamera();
  exportCal3_S2();
  export_geometry();

  exportSymbol();

  exportNoiseModels();

  exportValues();
  exportNonlinearFactor();
  exportNonlinearFactorGraph();
  exportLevenbergMarquardtOptimizer();
  exportISAM2();

  exportPriorFactors();
  exportBetweenFactors();
  exportGenericProjectionFactor();
  export_slam();

  exportImuFactor();
  exportScenario();
}

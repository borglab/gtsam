/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSerializationLinear.cpp
 * @brief 
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

/* ************************************************************************* */
// Noise model components
/* ************************************************************************* */

/* ************************************************************************* */
// Export Noisemodels
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");

/* ************************************************************************* */
// example noise models
static noiseModel::Diagonal::shared_ptr diag3 = noiseModel::Diagonal::Sigmas(Vector_(3, 0.1, 0.2, 0.3));
static noiseModel::Gaussian::shared_ptr gaussian3 = noiseModel::Gaussian::SqrtInformation(2.0 * eye(3,3));
static noiseModel::Isotropic::shared_ptr iso3 = noiseModel::Isotropic::Sigma(3, 0.2);
static noiseModel::Constrained::shared_ptr constrained3 = noiseModel::Constrained::MixedSigmas(Vector_(3, 0.0, 0.0, 0.1));
static noiseModel::Unit::shared_ptr unit3 = noiseModel::Unit::Create(3);

/* ************************************************************************* */
TEST (Serialization, noiseModels) {
  // tests using pointers to the derived class
  EXPECT(   equalsDereferenced<noiseModel::Diagonal::shared_ptr>(diag3));
  EXPECT(equalsDereferencedXML<noiseModel::Diagonal::shared_ptr>(diag3));

  EXPECT(   equalsDereferenced<noiseModel::Gaussian::shared_ptr>(gaussian3));
  EXPECT(equalsDereferencedXML<noiseModel::Gaussian::shared_ptr>(gaussian3));

  EXPECT(   equalsDereferenced<noiseModel::Isotropic::shared_ptr>(iso3));
  EXPECT(equalsDereferencedXML<noiseModel::Isotropic::shared_ptr>(iso3));

  EXPECT(   equalsDereferenced<noiseModel::Constrained::shared_ptr>(constrained3));
  EXPECT(equalsDereferencedXML<noiseModel::Constrained::shared_ptr>(constrained3));

  EXPECT(   equalsDereferenced<noiseModel::Unit::shared_ptr>(unit3));
  EXPECT(equalsDereferencedXML<noiseModel::Unit::shared_ptr>(unit3));
}

/* ************************************************************************* */
TEST (Serialization, SharedNoiseModel_noiseModels) {
  SharedNoiseModel diag3_sg = diag3;
  EXPECT(equalsDereferenced<SharedNoiseModel>(diag3_sg));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(diag3_sg));

  EXPECT(equalsDereferenced<SharedNoiseModel>(diag3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(diag3));

  EXPECT(equalsDereferenced<SharedNoiseModel>(iso3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(iso3));

  EXPECT(equalsDereferenced<SharedNoiseModel>(gaussian3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(gaussian3));

  EXPECT(equalsDereferenced<SharedNoiseModel>(unit3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(unit3));

  EXPECT(equalsDereferenced<SharedNoiseModel>(constrained3));
  EXPECT(equalsDereferencedXML<SharedNoiseModel>(constrained3));
}

/* ************************************************************************* */
TEST (Serialization, SharedDiagonal_noiseModels) {
  EXPECT(equalsDereferenced<SharedDiagonal>(diag3));
  EXPECT(equalsDereferencedXML<SharedDiagonal>(diag3));

  EXPECT(equalsDereferenced<SharedDiagonal>(iso3));
  EXPECT(equalsDereferencedXML<SharedDiagonal>(iso3));

  EXPECT(equalsDereferenced<SharedDiagonal>(unit3));
  EXPECT(equalsDereferencedXML<SharedDiagonal>(unit3));

  EXPECT(equalsDereferenced<SharedDiagonal>(constrained3));
  EXPECT(equalsDereferencedXML<SharedDiagonal>(constrained3));
}

/* ************************************************************************* */
// Linear components
/* ************************************************************************* */

/* Create GUIDs for factors */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor , "gtsam::HessianFactor");

/* ************************************************************************* */
TEST (Serialization, linear_factors) {
  VectorValues values;
  values.insert(0, Vector_(1, 1.0));
  values.insert(1, Vector_(2, 2.0,3.0));
  values.insert(2, Vector_(2, 4.0,5.0));
  EXPECT(equalsObj<VectorValues>(values));
  EXPECT(equalsXML<VectorValues>(values));

  Index i1 = 4, i2 = 7;
  Matrix A1 = eye(3), A2 = -1.0 * eye(3);
  Vector b = ones(3);
  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector_(3, 1.0, 2.0, 3.0));
  JacobianFactor jacobianfactor(i1, A1, i2, A2, b, model);
  EXPECT(equalsObj(jacobianfactor));
  EXPECT(equalsXML(jacobianfactor));

  HessianFactor hessianfactor(jacobianfactor);
  EXPECT(equalsObj(hessianfactor));
  EXPECT(equalsXML(hessianfactor));
}

/* ************************************************************************* */
TEST (Serialization, gaussian_conditional) {
  Matrix A1 = Matrix_(2,2, 1., 2., 3., 4.);
  Matrix A2 = Matrix_(2,2, 6., 0.2, 8., 0.4);
  Matrix R = Matrix_(2,2, 0.1, 0.3, 0.0, 0.34);
  Vector d(2); d << 0.2, 0.5;
  GaussianConditional cg(0, d, R, 1, A1, 2, A2, ones(2));

  EXPECT(equalsObj(cg));
  EXPECT(equalsXML(cg));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

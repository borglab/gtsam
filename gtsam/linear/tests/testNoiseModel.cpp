/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testNoiseModel.cpp
 * @date Jan 13, 2010
 * @author Richard Roberts
 * @author Frank Dellaert
 * @author Fan Jiang
 */


#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <limits>

using namespace std;
using namespace gtsam;
using namespace noiseModel;

static const double kSigma = 2, kInverseSigma = 1.0 / kSigma,
                    kVariance = kSigma * kSigma, prc = 1.0 / kVariance;
static const Matrix R = I_3x3 * kInverseSigma;
static const Matrix kCovariance = I_3x3 * kVariance;
static const Vector3 kSigmas(kSigma, kSigma, kSigma);

/* ************************************************************************* */
TEST(NoiseModel, constructors)
{
  Vector whitened = Vector3(5.0,10.0,15.0);
  Vector unwhitened = Vector3(10.0,20.0,30.0);

  // Construct noise models
  vector<Gaussian::shared_ptr> m;
  m.push_back(Gaussian::SqrtInformation(R,false));
  m.push_back(Gaussian::Covariance(kCovariance,false));
  m.push_back(Gaussian::Information(kCovariance.inverse(),false));
  m.push_back(Diagonal::Sigmas(kSigmas,false));
  m.push_back(Diagonal::Variances((Vector3(kVariance, kVariance, kVariance)),false));
  m.push_back(Diagonal::Precisions(Vector3(prc, prc, prc),false));
  m.push_back(Isotropic::Sigma(3, kSigma,false));
  m.push_back(Isotropic::Variance(3, kVariance,false));
  m.push_back(Isotropic::Precision(3, prc,false));

  // test kSigmas
  for(Gaussian::shared_ptr mi: m)
    EXPECT(assert_equal(kSigmas,mi->sigmas()));

  // test whiten
  for(Gaussian::shared_ptr mi: m)
    EXPECT(assert_equal(whitened,mi->whiten(unwhitened)));

  // test unwhiten
  for(Gaussian::shared_ptr mi: m)
    EXPECT(assert_equal(unwhitened,mi->unwhiten(whitened)));

  // test squared Mahalanobis distance
  double distance = 5*5+10*10+15*15;
  for(Gaussian::shared_ptr mi: m)
    DOUBLES_EQUAL(distance,mi->squaredMahalanobisDistance(unwhitened),1e-9);

  // test R matrix
  for(Gaussian::shared_ptr mi: m)
    EXPECT(assert_equal(R,mi->R()));

  // test covariance
  for(Gaussian::shared_ptr mi: m)
    EXPECT(assert_equal(kCovariance,mi->covariance()));

  // test covariance
  for(Gaussian::shared_ptr mi: m)
    EXPECT(assert_equal(kCovariance.inverse(),mi->information()));

  // test Whiten operator
  Matrix H((Matrix(3, 4) <<
      0.0, 0.0, 1.0, 1.0,
      0.0, 1.0, 0.0, 1.0,
      1.0, 0.0, 0.0, 1.0).finished());
  Matrix expected = kInverseSigma * H;
  for(Gaussian::shared_ptr mi: m)
    EXPECT(assert_equal(expected,mi->Whiten(H)));

  // can only test inplace version once :-)
  m[0]->WhitenInPlace(H);
  EXPECT(assert_equal(expected,H));
}

/* ************************************************************************* */
TEST(NoiseModel, Unit)
{
  Vector v = Vector3(5.0,10.0,15.0);
  Gaussian::shared_ptr u(Unit::Create(3));
  EXPECT(assert_equal(v,u->whiten(v)));
}

/* ************************************************************************* */
TEST(NoiseModel, equals)
{
  Gaussian::shared_ptr g1 = Gaussian::SqrtInformation(R),
                       g2 = Gaussian::SqrtInformation(I_3x3);
  Diagonal::shared_ptr d1 = Diagonal::Sigmas(Vector3(kSigma, kSigma, kSigma)),
                       d2 = Diagonal::Sigmas(Vector3(0.1, 0.2, 0.3));
  Isotropic::shared_ptr i1 = Isotropic::Sigma(3, kSigma),
                        i2 = Isotropic::Sigma(3, 0.7);

  EXPECT(assert_equal(*g1,*g1));
  EXPECT(assert_inequal(*g1, *g2));

  EXPECT(assert_equal(*d1,*d1));
  EXPECT(assert_inequal(*d1,*d2));

  EXPECT(assert_equal(*i1,*i1));
  EXPECT(assert_inequal(*i1,*i2));
}

// TODO enable test once a mechanism for smart constraints exists
///* ************************************************************************* */
//TEST(NoiseModel, ConstrainedSmart )
//{
//  Gaussian::shared_ptr nonconstrained = Constrained::MixedSigmas((Vector3(sigma, 0.0, sigma), true);
//  Diagonal::shared_ptr n1 = std::dynamic_pointer_cast<Diagonal>(nonconstrained);
//  Constrained::shared_ptr n2 = std::dynamic_pointer_cast<Constrained>(nonconstrained);
//  EXPECT(n1);
//  EXPECT(!n2);
//
//  Gaussian::shared_ptr constrained = Constrained::MixedSigmas(zero(3), true);
//  Diagonal::shared_ptr c1 = std::dynamic_pointer_cast<Diagonal>(constrained);
//  Constrained::shared_ptr c2 = std::dynamic_pointer_cast<Constrained>(constrained);
//  EXPECT(c1);
//  EXPECT(c2);
//}

/* ************************************************************************* */
TEST(NoiseModel, ConstrainedConstructors )
{
  Constrained::shared_ptr actual;
  size_t d = 3;
  double m = 100.0;
  Vector3 sigmas(kSigma, 0.0, 0.0);
  Vector3 mu(200.0, 300.0, 400.0);
  actual = Constrained::All(d);
  // TODO: why should this be a thousand ??? Dummy variable?
  EXPECT(assert_equal(Vector::Constant(d, 1000.0), actual->mu()));
  EXPECT(assert_equal(Vector::Constant(d, 0), actual->sigmas()));
  EXPECT(assert_equal(Vector::Constant(d, 0), actual->invsigmas())); // Actually zero as dummy value
  EXPECT(assert_equal(Vector::Constant(d, 0), actual->precisions())); // Actually zero as dummy value

  actual = Constrained::All(d, m);
  EXPECT(assert_equal(Vector::Constant(d, m), actual->mu()));

  actual = Constrained::All(d, mu);
  EXPECT(assert_equal(mu, actual->mu()));

  actual = Constrained::MixedSigmas(mu, sigmas);
  EXPECT(assert_equal(mu, actual->mu()));

  actual = Constrained::MixedSigmas(m, sigmas);
  EXPECT(assert_equal(Vector::Constant(d, m), actual->mu()));
}

/* ************************************************************************* */
TEST(NoiseModel, ConstrainedMixed )
{
  Vector feasible = Vector3(1.0, 0.0, 1.0),
      infeasible = Vector3(1.0, 1.0, 1.0);
  Diagonal::shared_ptr d = Constrained::MixedSigmas(Vector3(kSigma, 0.0, kSigma));
  // NOTE: we catch constrained variables elsewhere, so whitening does nothing
  EXPECT(assert_equal(Vector3(0.5, 1.0, 0.5),d->whiten(infeasible)));
  EXPECT(assert_equal(Vector3(0.5, 0.0, 0.5),d->whiten(feasible)));

  DOUBLES_EQUAL(0.5 * (1000.0 + 0.25 + 0.25),d->loss(d->squaredMahalanobisDistance(infeasible)),1e-9);
  DOUBLES_EQUAL(0.5, d->squaredMahalanobisDistance(feasible),1e-9);
  DOUBLES_EQUAL(0.5 * 0.5, d->loss(0.5),1e-9);
}

/* ************************************************************************* */
TEST(NoiseModel, ConstrainedAll )
{
  Vector feasible = Vector3(0.0, 0.0, 0.0),
       infeasible = Vector3(1.0, 1.0, 1.0);

  Constrained::shared_ptr i = Constrained::All(3);
  // NOTE: we catch constrained variables elsewhere, so whitening does nothing
  EXPECT(assert_equal(Vector3(1.0, 1.0, 1.0),i->whiten(infeasible)));
  EXPECT(assert_equal(Vector3(0.0, 0.0, 0.0),i->whiten(feasible)));

  DOUBLES_EQUAL(0.5 * 1000.0 * 3.0,i->loss(i->squaredMahalanobisDistance(infeasible)),1e-9);
  DOUBLES_EQUAL(0.0, i->squaredMahalanobisDistance(feasible), 1e-9);
  DOUBLES_EQUAL(0.0, i->loss(0.0),1e-9);
}

/* ************************************************************************* */
namespace exampleQR {
  // create a matrix to eliminate
  Matrix Ab = (Matrix(4, 7) <<
      -1.,  0.,  1.,  0.,  0.,  0., -0.2,
      0., -1.,  0.,  1.,  0.,  0.,  0.3,
      1.,  0.,  0.,  0., -1.,  0.,  0.2,
      0.,  1.,  0.,  0.,  0., -1., -0.1).finished();
  Vector sigmas = (Vector(4) << 0.2, 0.2, 0.1, 0.1).finished();

  // the matrix AB yields the following factorized version:
  Matrix Rd = (Matrix(4, 7) <<
      11.1803,   0.0,   -2.23607, 0.0,    -8.94427, 0.0,     2.23607,
      0.0,   11.1803,    0.0,    -2.23607, 0.0,    -8.94427,-1.56525,
      0.0,       0.0,    4.47214, 0.0,    -4.47214, 0.0,     0.0,
      0.0,       0.0,   0.0,     4.47214, 0.0,    -4.47214, 0.894427).finished();

  SharedDiagonal diagonal = noiseModel::Diagonal::Sigmas(sigmas);
}

/* ************************************************************************* */
TEST( NoiseModel, QR )
{
  Matrix Ab1 = exampleQR::Ab;
  Matrix Ab2 = exampleQR::Ab; // otherwise overwritten !

  // Call Gaussian version
  SharedDiagonal actual1 = exampleQR::diagonal->QR(Ab1);
  EXPECT(actual1->isUnit());
  EXPECT(linear_dependent(exampleQR::Rd,Ab1,1e-4)); // Ab was modified in place !!!

  // Expected result for constrained version
  Vector expectedSigmas = (Vector(4) << 0.0894427, 0.0894427, 0.223607, 0.223607).finished();
  SharedDiagonal expectedModel = noiseModel::Diagonal::Sigmas(expectedSigmas);
  Matrix expectedRd2 = (Matrix(4, 7) <<
      1.,  0., -0.2,  0., -0.8, 0.,  0.2,
      0.,  1.,  0.,-0.2,   0., -0.8,-0.14,
      0.,  0.,  1.,   0., -1.,  0.,  0.0,
      0.,  0.,  0.,   1.,  0., -1.,  0.2).finished();

  // Call Constrained version
  SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(exampleQR::sigmas);
  SharedDiagonal actual2 = constrained->QR(Ab2);
  EXPECT(assert_equal(*expectedModel, *actual2, 1e-6));
  EXPECT(linear_dependent(expectedRd2, Ab2, 1e-6));  // Ab was modified in place !!!
}

/* ************************************************************************* */
TEST(NoiseModel, OverdeterminedQR) {
  Matrix Ab1(9, 4);
  Ab1 << 0, 1, 0, 0,  //
      0, 0, 1, 0,    //
      Matrix74::Ones();
  Matrix Ab2 = Ab1; // otherwise overwritten !

  // Call Gaussian version
  Vector9 sigmas = Vector9::Ones() ;
  SharedDiagonal diagonal = noiseModel::Diagonal::Sigmas(sigmas);
  SharedDiagonal actual1 = diagonal->QR(Ab1);
  EXPECT(actual1->isUnit());
  Matrix expectedRd(9,4);
  expectedRd << -2.64575131, -2.64575131, -2.64575131, -2.64575131,  //
      0.0, -1, 0, 0,                                                 //
      0.0, 0.0, -1, 0,                                               //
      Matrix64::Zero();
  EXPECT(assert_equal(expectedRd, Ab1, 1e-4));  // Ab was modified in place !!!

  // Expected result for constrained version
  Vector3 expectedSigmas(0.377964473, 1, 1);
  SharedDiagonal expectedModel = noiseModel::Diagonal::Sigmas(expectedSigmas);

  // Call Constrained version
  SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(sigmas);
  SharedDiagonal actual2 = constrained->QR(Ab2);
  EXPECT(assert_equal(*expectedModel, *actual2, 1e-6));
  expectedRd.row(0) *= 0.377964473; // not divided by sigma!
  EXPECT(assert_equal(-expectedRd, Ab2, 1e-6));  // Ab was modified in place !!!
}

/* ************************************************************************* */
TEST( NoiseModel, MixedQR )
{
  // Call Constrained version, with first and third row treated as constraints
  // Naming the 6 variables u,v,w,x,y,z, we have
  // u = -z
  // w = -x
  // And let's have simple priors on variables
  Matrix Ab(5,6+1);
  Ab <<
      1,0,0,0,0,1,  0, // u+z = 0
      0,0,0,0,1,0,  0, // y^2
      0,0,1,1,0,0,  0, // w+x = 0
      0,1,0,0,0,0,  0, // v^2
      0,0,0,0,0,1,  0; // z^2
  Vector mixed_sigmas = (Vector(5) << 0, 1, 0, 1, 1).finished();
  SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(mixed_sigmas);

  // Expected result
  Vector expectedSigmas = (Vector(5) << 0, 1, 0, 1, 1).finished();
  SharedDiagonal expectedModel = noiseModel::Diagonal::Sigmas(expectedSigmas);
  Matrix expectedRd(5, 6+1);
  expectedRd << 1, 0, 0, 0, 0, 1, 0,  //
                0, 1, 0, 0, 0, 0, 0,  //
                0, 0, 1, 1, 0, 0, 0,  //
                0, 0, 0, 0, 1, 0, 0,  //
                0, 0, 0, 0, 0, 1, 0;  //

  SharedDiagonal actual = constrained->QR(Ab);
  EXPECT(assert_equal(*expectedModel,*actual,1e-6));
  EXPECT(linear_dependent(expectedRd,Ab,1e-6)); // Ab was modified in place !!!
}

/* ************************************************************************* */
TEST( NoiseModel, MixedQR2 )
{
  // Let's have three variables x,y,z, but x=z and y=z
  // Hence, all non-constraints are really measurements on z
  Matrix Ab(11,3+1);
  Ab <<
      1,0,0,  0, //
      0,1,0,  0, //
      0,0,1,  0, //
     -1,0,1,  0, // x=z
      1,0,0,  0, //
      0,1,0,  0, //
      0,0,1,  0, //
     0,-1,1,  0, // y=z
      1,0,0,  0, //
      0,1,0,  0, //
      0,0,1,  0; //

  Vector sigmas(11);
  sigmas.setOnes();
  sigmas[3] = 0;
  sigmas[7] = 0;
  SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(sigmas);

  // Expected result
  Vector3 expectedSigmas(0,0,1.0/3);
  SharedDiagonal expectedModel = noiseModel::Constrained::MixedSigmas(expectedSigmas);
  Matrix expectedRd(11, 3+1);
  expectedRd.setZero();
  expectedRd.row(0) << -1,  0, 1,  0;  // x=z
  expectedRd.row(1) <<  0, -1, 1,  0;  // y=z
  expectedRd.row(2) <<  0,  0, 1,  0;  // z=0 +/- 1/3

  SharedDiagonal actual = constrained->QR(Ab);
  EXPECT(assert_equal(*expectedModel,*actual,1e-6));
  EXPECT(assert_equal(expectedRd,Ab,1e-6)); // Ab was modified in place !!!
}

/* ************************************************************************* */
TEST( NoiseModel, FullyConstrained )
{
  Matrix Ab(3,7);
  Ab <<
      1,0,0,0,0,1,  2, // u+z = 2
      0,0,1,1,0,0,  4, // w+x = 4
      0,1,0,1,1,1,  8; // v+x+y+z=8
  SharedDiagonal constrained = noiseModel::Constrained::All(3);

  // Expected result
  SharedDiagonal expectedModel = noiseModel::Diagonal::Sigmas(Vector3 (0,0,0));
  Matrix expectedRd(3, 7);
  expectedRd << 1, 0, 0, 0, 0, 1, 2,  //
                0, 1, 0, 1, 1, 1, 8,  //
                0, 0, 1, 1, 0, 0, 4;  //

  SharedDiagonal actual = constrained->QR(Ab);
  EXPECT(assert_equal(*expectedModel,*actual,1e-6));
  EXPECT(linear_dependent(expectedRd,Ab,1e-6)); // Ab was modified in place !!!
}

/* ************************************************************************* */
// This matches constraint_eliminate2 in testJacobianFactor
TEST(NoiseModel, QRNan )
{
  SharedDiagonal constrained = noiseModel::Constrained::All(2);
  Matrix Ab = (Matrix25() << 2, 4, 2, 4, 6,   2, 1, 2, 4, 4).finished();

  SharedDiagonal expected = noiseModel::Constrained::All(2);
  Matrix expectedAb = (Matrix25() << 1, 2, 1, 2, 3, 0, 1, 0, 0, 2.0/3).finished();

  SharedDiagonal actual = constrained->QR(Ab);
  EXPECT(assert_equal(*expected,*actual));
  EXPECT(linear_dependent(expectedAb,Ab));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartSqrtInformation )
{
  bool smart = true;
  gtsam::SharedGaussian expected = Unit::Create(3);
  gtsam::SharedGaussian actual = Gaussian::SqrtInformation(I_3x3, smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartSqrtInformation2 )
{
  bool smart = true;
  gtsam::SharedGaussian expected = Unit::Isotropic::Sigma(3,2);
  gtsam::SharedGaussian actual = Gaussian::SqrtInformation(0.5*I_3x3, smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartInformation )
{
  bool smart = true;
  gtsam::SharedGaussian expected = Unit::Isotropic::Variance(3,2);
  Matrix M = 0.5*I_3x3;
  EXPECT(checkIfDiagonal(M));
  gtsam::SharedGaussian actual = Gaussian::Information(M, smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartCovariance )
{
  bool smart = true;
  gtsam::SharedGaussian expected = Unit::Create(3);
  gtsam::SharedGaussian actual = Gaussian::Covariance(I_3x3, smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, ScalarOrVector )
{
  bool smart = true;
  SharedGaussian expected = Unit::Create(3);
  SharedGaussian actual = Gaussian::Covariance(I_3x3, smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, WhitenInPlace)
{
  Vector sigmas = Vector3(0.1, 0.1, 0.1);
  SharedDiagonal model = Diagonal::Sigmas(sigmas);
  Matrix A = I_3x3;
  model->WhitenInPlace(A);
  Matrix expected = I_3x3 * 10;
  EXPECT(assert_equal(expected, A));
}

/* ************************************************************************* */

/*
 * These tests are responsible for testing the weight functions for the m-estimators in GTSAM.
 * The weight function is related to the analytic derivative of the loss function. See
 *  https://members.loria.fr/MOBerger/Enseignement/Master2/Documents/ZhangIVC-97-01.pdf
 * for details. This weight function is required when optimizing cost functions with robust
 * penalties using iteratively re-weighted least squares.
 */

TEST(NoiseModel, robustFunctionFair)
{
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0, error4 = -1.0;
  const mEstimator::Fair::shared_ptr fair = mEstimator::Fair::Create(k);
  DOUBLES_EQUAL(0.8333333333333333, fair->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.3333333333333333, fair->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.3333333333333333, fair->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.8333333333333333, fair->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.441961080151135, fair->loss(error1), 1e-8);
  DOUBLES_EQUAL(22.534692783297260, fair->loss(error2), 1e-8);
  DOUBLES_EQUAL(22.534692783297260, fair->loss(error3), 1e-8);
  DOUBLES_EQUAL(0.441961080151135, fair->loss(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionHuber)
{
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0, error4 = -1.0;
  const mEstimator::Huber::shared_ptr huber = mEstimator::Huber::Create(k);
  DOUBLES_EQUAL(1.0, huber->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.5, huber->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.5, huber->weight(error3), 1e-8);
  DOUBLES_EQUAL(1.0, huber->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.5000, huber->loss(error1), 1e-8);
  DOUBLES_EQUAL(37.5000, huber->loss(error2), 1e-8);
  DOUBLES_EQUAL(37.5000, huber->loss(error3), 1e-8);
  DOUBLES_EQUAL(0.5000, huber->loss(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionCauchy)
{
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0, error4 = -1.0;
  const mEstimator::Cauchy::shared_ptr cauchy = mEstimator::Cauchy::Create(k);
  DOUBLES_EQUAL(0.961538461538461, cauchy->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.2000, cauchy->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.2000, cauchy->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.961538461538461, cauchy->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.490258914416017, cauchy->loss(error1), 1e-8);
  DOUBLES_EQUAL(20.117973905426254, cauchy->loss(error2), 1e-8);
  DOUBLES_EQUAL(20.117973905426254, cauchy->loss(error3), 1e-8);
  DOUBLES_EQUAL(0.490258914416017, cauchy->loss(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionAsymmetricCauchy)
{
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0, error4 = -1.0;
  const mEstimator::AsymmetricCauchy::shared_ptr cauchy = mEstimator::AsymmetricCauchy::Create(k);
  DOUBLES_EQUAL(0.961538461538461, cauchy->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.2000, cauchy->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(1.0, cauchy->weight(error3), 1e-8);
  DOUBLES_EQUAL(1.0, cauchy->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.490258914416017, cauchy->loss(error1), 1e-8);
  DOUBLES_EQUAL(20.117973905426254, cauchy->loss(error2), 1e-8);
  DOUBLES_EQUAL(50.0, cauchy->loss(error3), 1e-8);
  DOUBLES_EQUAL(0.5, cauchy->loss(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionGemanMcClure)
{
  const double k = 1.0, error1 = 1.0, error2 = 10.0, error3 = -10.0, error4 = -1.0;
  const mEstimator::GemanMcClure::shared_ptr gmc = mEstimator::GemanMcClure::Create(k);
  DOUBLES_EQUAL(0.25      , gmc->weight(error1), 1e-8);
  DOUBLES_EQUAL(9.80296e-5, gmc->weight(error2), 1e-8);
  DOUBLES_EQUAL(9.80296e-5, gmc->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.25      , gmc->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.2500, gmc->loss(error1), 1e-8);
  DOUBLES_EQUAL(0.495049504950495, gmc->loss(error2), 1e-8);
  DOUBLES_EQUAL(0.495049504950495, gmc->loss(error3), 1e-8);
  DOUBLES_EQUAL(0.2500, gmc->loss(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionWelsch)
{
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0, error4 = -1.0;
  const mEstimator::Welsch::shared_ptr welsch = mEstimator::Welsch::Create(k);
  DOUBLES_EQUAL(0.960789439152323, welsch->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.018315638888734, welsch->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.018315638888734, welsch->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.960789439152323, welsch->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.490132010595960, welsch->loss(error1), 1e-8);
  DOUBLES_EQUAL(12.271054513890823, welsch->loss(error2), 1e-8);
  DOUBLES_EQUAL(12.271054513890823, welsch->loss(error3), 1e-8);
  DOUBLES_EQUAL(0.490132010595960, welsch->loss(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionTukey)
{
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0, error4 = -1.0;
  const mEstimator::Tukey::shared_ptr tukey = mEstimator::Tukey::Create(k);
  DOUBLES_EQUAL(0.9216, tukey->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.0, tukey->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.0, tukey->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.9216, tukey->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.480266666666667, tukey->loss(error1), 1e-8);
  DOUBLES_EQUAL(4.166666666666667, tukey->loss(error2), 1e-8);
  DOUBLES_EQUAL(4.166666666666667, tukey->loss(error3), 1e-8);
  DOUBLES_EQUAL(0.480266666666667, tukey->loss(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionAsymmetricTukey)
{
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0, error4 = -1.0;
  const mEstimator::AsymmetricTukey::shared_ptr tukey = mEstimator::AsymmetricTukey::Create(k);
  DOUBLES_EQUAL(0.9216, tukey->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.0, tukey->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(1.0, tukey->weight(error3), 1e-8);
  DOUBLES_EQUAL(1.0, tukey->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.480266666666667, tukey->loss(error1), 1e-8);
  DOUBLES_EQUAL(4.166666666666667, tukey->loss(error2), 1e-8);
  DOUBLES_EQUAL(50.0, tukey->loss(error3), 1e-8);
  DOUBLES_EQUAL(0.5, tukey->loss(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionDCS)
{
  const double k = 1.0, error1 = 1.0, error2 = 10.0;
  const mEstimator::DCS::shared_ptr dcs = mEstimator::DCS::Create(k);

  DOUBLES_EQUAL(1.0       , dcs->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.00039211, dcs->weight(error2), 1e-8);

  DOUBLES_EQUAL(0.5         , dcs->loss(error1), 1e-8);
  DOUBLES_EQUAL(0.9900990099, dcs->loss(error2), 1e-8);
}

TEST(NoiseModel, robustFunctionL2WithDeadZone)
{
  const double k = 1.0, e0 = -10.0, e1 = -1.01, e2 = -0.99, e3 = 0.99, e4 = 1.01, e5 = 10.0;
  const mEstimator::L2WithDeadZone::shared_ptr lsdz = mEstimator::L2WithDeadZone::Create(k);

  DOUBLES_EQUAL(0.9,           lsdz->weight(e0), 1e-8);
  DOUBLES_EQUAL(0.00990099009, lsdz->weight(e1), 1e-8);
  DOUBLES_EQUAL(0.0,           lsdz->weight(e2), 1e-8);
  DOUBLES_EQUAL(0.0,           lsdz->weight(e3), 1e-8);
  DOUBLES_EQUAL(0.00990099009, lsdz->weight(e4), 1e-8);
  DOUBLES_EQUAL(0.9,           lsdz->weight(e5), 1e-8);

  DOUBLES_EQUAL(40.5,    lsdz->loss(e0), 1e-8);
  DOUBLES_EQUAL(0.00005, lsdz->loss(e1), 1e-8);
  DOUBLES_EQUAL(0.0,     lsdz->loss(e2), 1e-8);
  DOUBLES_EQUAL(0.0,     lsdz->loss(e3), 1e-8);
  DOUBLES_EQUAL(0.00005, lsdz->loss(e4), 1e-8);
  DOUBLES_EQUAL(40.5,    lsdz->loss(e5), 1e-8);
}

/* ************************************************************************* */
TEST(NoiseModel, robustNoiseHuber)
{
  const double k = 10.0, error1 = 1.0, error2 = 100.0;
  Matrix A = (Matrix(2, 2) << 1.0, 10.0, 100.0, 1000.0).finished();
  Vector b = Vector2(error1, error2);
  const Robust::shared_ptr robust = Robust::Create(
    mEstimator::Huber::Create(k, mEstimator::Huber::Scalar),
    Unit::Create(2));

  robust->WhitenSystem(A, b);

  DOUBLES_EQUAL(error1, b(0), 1e-8);
  DOUBLES_EQUAL(sqrt(k*error2), b(1), 1e-8);

  DOUBLES_EQUAL(1.0, A(0,0), 1e-8);
  DOUBLES_EQUAL(10.0, A(0,1), 1e-8);
  DOUBLES_EQUAL(sqrt(k*100.0), A(1,0), 1e-8);
  DOUBLES_EQUAL(sqrt(k/100.0)*1000.0, A(1,1), 1e-8);
}

TEST(NoiseModel, robustNoiseGemanMcClure)
{
  const double k = 1.0, error1 = 1.0, error2 = 100.0;
  const double a00 = 1.0, a01 = 10.0, a10 = 100.0, a11 = 1000.0;
  Matrix A = (Matrix(2, 2) << a00, a01, a10, a11).finished();
  Vector b = Vector2(error1, error2);
  const Robust::shared_ptr robust = Robust::Create(
    mEstimator::GemanMcClure::Create(k, mEstimator::GemanMcClure::Scalar),
    Unit::Create(2));

  robust->WhitenSystem(A, b);

  const double k2 = k*k;
  const double k4 = k2*k2;
  const double k2error = k2 + error2*error2;

  const double sqrt_weight_error1 = sqrt(0.25);
  const double sqrt_weight_error2 = sqrt(k4/(k2error*k2error));

  DOUBLES_EQUAL(sqrt_weight_error1*error1, b(0), 1e-8);
  DOUBLES_EQUAL(sqrt_weight_error2*error2, b(1), 1e-8);

  DOUBLES_EQUAL(sqrt_weight_error1*a00, A(0,0), 1e-8);
  DOUBLES_EQUAL(sqrt_weight_error1*a01, A(0,1), 1e-8);
  DOUBLES_EQUAL(sqrt_weight_error2*a10, A(1,0), 1e-8);
  DOUBLES_EQUAL(sqrt_weight_error2*a11, A(1,1), 1e-8);
}

TEST(NoiseModel, robustNoiseDCS)
{
  const double k = 1.0, error1 = 1.0, error2 = 100.0;
  const double a00 = 1.0, a01 = 10.0, a10 = 100.0, a11 = 1000.0;
  Matrix A = (Matrix(2, 2) << a00, a01, a10, a11).finished();
  Vector b = Vector2(error1, error2);
  const Robust::shared_ptr robust = Robust::Create(
    mEstimator::DCS::Create(k, mEstimator::DCS::Scalar),
    Unit::Create(2));

  robust->WhitenSystem(A, b);

  const double sqrt_weight = 2.0*k/(k + error2*error2);

  DOUBLES_EQUAL(error1, b(0), 1e-8);
  DOUBLES_EQUAL(sqrt_weight*error2, b(1), 1e-8);

  DOUBLES_EQUAL(a00, A(0,0), 1e-8);
  DOUBLES_EQUAL(a01, A(0,1), 1e-8);
  DOUBLES_EQUAL(sqrt_weight*a10, A(1,0), 1e-8);
  DOUBLES_EQUAL(sqrt_weight*a11, A(1,1), 1e-8);
}

TEST(NoiseModel, robustNoiseL2WithDeadZone)
{
  double dead_zone_size = 1.0;
  auto robust = noiseModel::Robust::Create(
      noiseModel::mEstimator::L2WithDeadZone::Create(dead_zone_size),
      Unit::Create(3));

  for (int i = 0; i < 5; i++) {
    Vector error = Vector3(i, 0, 0);
    robust->WhitenSystem(error);
    DOUBLES_EQUAL(std::fmax(0, i - dead_zone_size) * i,
                  robust->squaredMahalanobisDistance(error), 1e-8);
  }
}

/* ************************************************************************* */
TEST(NoiseModel, robustNoiseCustomHuber) {
  const double k = 10.0, error1 = 1.0, error2 = 100.0;
  Matrix A = (Matrix(2, 2) << 1.0, 10.0, 100.0, 1000.0).finished();
  Vector b = Vector2(error1, error2);
  const Robust::shared_ptr robust =
      Robust::Create(mEstimator::Custom::Create(
                         [k](double e) {
                           const auto abs_e = std::abs(e);
                           return abs_e <= k ? 1.0 : k / abs_e;
                         },
                         [k](double e) {
                           const auto abs_e = std::abs(e);
                           return abs_e <= k ? abs_e * abs_e / 2.0 : k * abs_e - k * k / 2.0;
                         },
                         noiseModel::mEstimator::Custom::Scalar, "Huber"),
                     Unit::Create(2));

  robust->WhitenSystem(A, b);

  DOUBLES_EQUAL(error1, b(0), 1e-8);
  DOUBLES_EQUAL(sqrt(k * error2), b(1), 1e-8);

  DOUBLES_EQUAL(1.0, A(0, 0), 1e-8);
  DOUBLES_EQUAL(10.0, A(0, 1), 1e-8);
  DOUBLES_EQUAL(sqrt(k * 100.0), A(1, 0), 1e-8);
  DOUBLES_EQUAL(sqrt(k / 100.0) * 1000.0, A(1, 1), 1e-8);
}

TEST(NoiseModel, lossFunctionAtZero)
{
  const double k = 5.0;
  auto fair = mEstimator::Fair::Create(k);
  DOUBLES_EQUAL(fair->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(fair->weight(0), 1, 1e-8);
  auto huber = mEstimator::Huber::Create(k);
  DOUBLES_EQUAL(huber->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(huber->weight(0), 1, 1e-8);
  auto cauchy = mEstimator::Cauchy::Create(k);
  DOUBLES_EQUAL(cauchy->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(cauchy->weight(0), 1, 1e-8);
  auto gmc = mEstimator::GemanMcClure::Create(k);
  DOUBLES_EQUAL(gmc->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(gmc->weight(0), 1, 1e-8);
  auto welsch = mEstimator::Welsch::Create(k);
  DOUBLES_EQUAL(welsch->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(welsch->weight(0), 1, 1e-8);
  auto tukey = mEstimator::Tukey::Create(k);
  DOUBLES_EQUAL(tukey->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(tukey->weight(0), 1, 1e-8);
  auto dcs = mEstimator::DCS::Create(k);
  DOUBLES_EQUAL(dcs->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(dcs->weight(0), 1, 1e-8);
  auto lsdz = mEstimator::L2WithDeadZone::Create(k);
  DOUBLES_EQUAL(lsdz->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(lsdz->weight(0), 0, 1e-8);
  auto assy_cauchy = mEstimator::AsymmetricCauchy::Create(k);
  DOUBLES_EQUAL(lsdz->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(lsdz->weight(0), 0, 1e-8);
  auto assy_tukey = mEstimator::AsymmetricTukey::Create(k);
  DOUBLES_EQUAL(lsdz->loss(0), 0, 1e-8);
  DOUBLES_EQUAL(lsdz->weight(0), 0, 1e-8);
}


/* ************************************************************************* */
#define TEST_GAUSSIAN(gaussian)\
  EQUALITY(info, gaussian->information());\
  EQUALITY(cov, gaussian->covariance());\
  EXPECT(assert_equal(white, gaussian->whiten(e)));\
  EXPECT(assert_equal(e, gaussian->unwhiten(white)));\
  EXPECT_DOUBLES_EQUAL(251.0, gaussian->squaredMahalanobisDistance(e), 1e-9);\
  EXPECT_DOUBLES_EQUAL(0.5 * 251.0, gaussian->loss(251.0), 1e-9);\
  Matrix A = R.inverse(); Vector b = e;\
  gaussian->WhitenSystem(A, b);\
  EXPECT(assert_equal(I, A));\
  EXPECT(assert_equal(white, b));

TEST(NoiseModel, NonDiagonalGaussian)
{
  Matrix3 R;
  R << 6, 5, 4, 0, 3, 2, 0, 0, 1;
  const Matrix3 info = R.transpose() * R;
  const Matrix3 cov = info.inverse();
  const Vector3 e(1, 1, 1), white = R * e;
  Matrix I = I_3x3;


  {
  SharedGaussian gaussian = Gaussian::SqrtInformation(R);
  TEST_GAUSSIAN(gaussian);
  }

  {
  SharedGaussian gaussian = Gaussian::Information(info);
  TEST_GAUSSIAN(gaussian);
  }

  {
  SharedGaussian gaussian = Gaussian::Covariance(cov);
  TEST_GAUSSIAN(gaussian);
  }
}

TEST(NoiseModel, ComputeLogNormalizerConstant) {
  // Very simple 1D noise model, which we can compute by hand.
  double sigma = 0.1;
  auto noise_model = Isotropic::Sigma(1, sigma);
  double actual_value = ComputeLogNormalizerConstant(noise_model);
  // Compute log(|2πΣ|) by hand.
  // = log(2π) + log(Σ) (since it is 1D)
  constexpr double log2pi = 1.8378770664093454835606594728112;
  double expected_value = log2pi + log(sigma * sigma);
  EXPECT_DOUBLES_EQUAL(expected_value, actual_value, 1e-9);

  // Similar situation in the 3D case
  size_t n = 3;
  auto noise_model2 = Isotropic::Sigma(n, sigma);
  double actual_value2 = ComputeLogNormalizerConstant(noise_model2);
  // We multiply by 3 due to the determinant
  double expected_value2 = n * (log2pi + log(sigma * sigma));
  EXPECT_DOUBLES_EQUAL(expected_value2, actual_value2, 1e-9);
}

/* ************************************************************************* */
int main() {  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

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
 */


#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>

#include <iostream>
#include <limits>

using namespace std;
using namespace gtsam;
using namespace noiseModel;
using namespace boost::assign;

static const double kSigma = 2, kInverseSigma = 1.0 / kSigma,
                    kVariance = kSigma * kSigma, prc = 1.0 / kVariance;
static const Matrix R = Matrix3::Identity() * kInverseSigma;
static const Matrix kCovariance = Matrix3::Identity() * kVariance;
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
  BOOST_FOREACH(Gaussian::shared_ptr mi, m)
    EXPECT(assert_equal(kSigmas,mi->sigmas()));

  // test whiten
  BOOST_FOREACH(Gaussian::shared_ptr mi, m)
    EXPECT(assert_equal(whitened,mi->whiten(unwhitened)));

  // test unwhiten
  BOOST_FOREACH(Gaussian::shared_ptr mi, m)
    EXPECT(assert_equal(unwhitened,mi->unwhiten(whitened)));

  // test Mahalanobis distance
  double distance = 5*5+10*10+15*15;
  BOOST_FOREACH(Gaussian::shared_ptr mi, m)
    DOUBLES_EQUAL(distance,mi->Mahalanobis(unwhitened),1e-9);

  // test R matrix
  BOOST_FOREACH(Gaussian::shared_ptr mi, m)
    EXPECT(assert_equal(R,mi->R()));

  // test covariance
  BOOST_FOREACH(Gaussian::shared_ptr mi, m)
    EXPECT(assert_equal(kCovariance,mi->covariance()));

  // test covariance
  BOOST_FOREACH(Gaussian::shared_ptr mi, m)
    EXPECT(assert_equal(kCovariance.inverse(),mi->information()));

  // test Whiten operator
  Matrix H((Matrix(3, 4) <<
      0.0, 0.0, 1.0, 1.0,
      0.0, 1.0, 0.0, 1.0,
      1.0, 0.0, 0.0, 1.0).finished());
  Matrix expected = kInverseSigma * H;
  BOOST_FOREACH(Gaussian::shared_ptr mi, m)
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
                       g2 = Gaussian::SqrtInformation(eye(3,3));
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
//  Diagonal::shared_ptr n1 = boost::dynamic_pointer_cast<Diagonal>(nonconstrained);
//  Constrained::shared_ptr n2 = boost::dynamic_pointer_cast<Constrained>(nonconstrained);
//  EXPECT(n1);
//  EXPECT(!n2);
//
//  Gaussian::shared_ptr constrained = Constrained::MixedSigmas(zero(3), true);
//  Diagonal::shared_ptr c1 = boost::dynamic_pointer_cast<Diagonal>(constrained);
//  Constrained::shared_ptr c2 = boost::dynamic_pointer_cast<Constrained>(constrained);
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
  EXPECT(assert_equal(gtsam::repeat(d, 1000.0), actual->mu()));
  EXPECT(assert_equal(gtsam::repeat(d, 0), actual->sigmas()));
  EXPECT(assert_equal(gtsam::repeat(d, 0), actual->invsigmas())); // Actually zero as dummy value
  EXPECT(assert_equal(gtsam::repeat(d, 0), actual->precisions())); // Actually zero as dummy value

  actual = Constrained::All(d, m);
  EXPECT(assert_equal(gtsam::repeat(d, m), actual->mu()));

  actual = Constrained::All(d, mu);
  EXPECT(assert_equal(mu, actual->mu()));

  actual = Constrained::MixedSigmas(mu, sigmas);
  EXPECT(assert_equal(mu, actual->mu()));

  actual = Constrained::MixedSigmas(m, sigmas);
  EXPECT(assert_equal( gtsam::repeat(d, m), actual->mu()));
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

  DOUBLES_EQUAL(1000.0 + 0.25 + 0.25,d->distance(infeasible),1e-9);
  DOUBLES_EQUAL(0.5,d->distance(feasible),1e-9);
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

  DOUBLES_EQUAL(1000.0 * 3.0,i->distance(infeasible),1e-9);
  DOUBLES_EQUAL(0.0,i->distance(feasible),1e-9);
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

TEST( NoiseModel, QR )
{
  Matrix Ab1 = exampleQR::Ab;
  Matrix Ab2 = exampleQR::Ab; // otherwise overwritten !

  // Expected result
  Vector expectedSigmas = (Vector(4) << 0.0894427, 0.0894427, 0.223607, 0.223607).finished();
  SharedDiagonal expectedModel = noiseModel::Diagonal::Sigmas(expectedSigmas);

  // Call Gaussian version
  SharedDiagonal actual1 = exampleQR::diagonal->QR(Ab1);
  EXPECT(!actual1);
  EXPECT(linear_dependent(exampleQR::Rd,Ab1,1e-4)); // Ab was modified in place !!!

  // Call Constrained version
  SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(exampleQR::sigmas);
  SharedDiagonal actual2 = constrained->QR(Ab2);
  SharedDiagonal expectedModel2 = noiseModel::Diagonal::Sigmas(expectedSigmas);
  EXPECT(assert_equal(*expectedModel2,*actual2,1e-6));
  Matrix expectedRd2 = (Matrix(4, 7) <<
      1.,  0., -0.2,  0., -0.8, 0.,  0.2,
      0.,  1.,  0.,-0.2,   0., -0.8,-0.14,
      0.,  0.,  1.,   0., -1.,  0.,  0.0,
      0.,  0.,  0.,   1.,  0., -1.,  0.2).finished();
  EXPECT(linear_dependent(expectedRd2,Ab2,1e-6)); // Ab was modified in place !!!
}

/* ************************************************************************* */
TEST(NoiseModel, QRNan )
{
  SharedDiagonal constrained = noiseModel::Constrained::All(2);
  Matrix Ab = (Matrix(2, 5) << 1., 2., 1., 2., 3., 2., 1., 2., 4., 4.).finished();

  SharedDiagonal expected = noiseModel::Constrained::All(2);
  Matrix expectedAb = (Matrix(2, 5) << 1., 2., 1., 2., 3., 0., 1., 0., 0., 2.0/3).finished();

  SharedDiagonal actual = constrained->QR(Ab);
  EXPECT(assert_equal(*expected,*actual));
  EXPECT(assert_equal(expectedAb,Ab));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartSqrtInformation )
{
  bool smart = true;
  gtsam::SharedGaussian expected = Unit::Create(3);
  gtsam::SharedGaussian actual = Gaussian::SqrtInformation(eye(3), smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartSqrtInformation2 )
{
  bool smart = true;
  gtsam::SharedGaussian expected = Unit::Isotropic::Sigma(3,2);
  gtsam::SharedGaussian actual = Gaussian::SqrtInformation(0.5*eye(3), smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartInformation )
{
  bool smart = true;
  gtsam::SharedGaussian expected = Unit::Isotropic::Variance(3,2);
  Matrix M = 0.5*eye(3);
  EXPECT(checkIfDiagonal(M));
  gtsam::SharedGaussian actual = Gaussian::Information(M, smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartCovariance )
{
  bool smart = true;
  gtsam::SharedGaussian expected = Unit::Create(3);
  gtsam::SharedGaussian actual = Gaussian::Covariance(eye(3), smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, ScalarOrVector )
{
  bool smart = true;
  SharedGaussian expected = Unit::Create(3);
  SharedGaussian actual = Gaussian::Covariance(eye(3), smart);
  EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, WhitenInPlace)
{
  Vector sigmas = Vector3(0.1, 0.1, 0.1);
  SharedDiagonal model = Diagonal::Sigmas(sigmas);
  Matrix A = eye(3);
  model->WhitenInPlace(A);
  Matrix expected = eye(3) * 10;
  EXPECT(assert_equal(expected, A));
}

/* ************************************************************************* */
TEST(NoiseModel, robustFunctionHuber)
{
  const double k = 5.0, error1 = 1.0, error2 = 10.0;
  const mEstimator::Huber::shared_ptr huber = mEstimator::Huber::Create(k);
  const double weight1 = huber->weight(error1),
               weight2 = huber->weight(error2);
  DOUBLES_EQUAL(1.0, weight1, 1e-8);
  DOUBLES_EQUAL(0.5, weight2, 1e-8);
}

TEST(NoiseModel, robustFunctionGemanMcClure)
{
  const double k = 1.0, error1 = 1.0, error2 = 10.0;
  const mEstimator::GemanMcClure::shared_ptr gmc = mEstimator::GemanMcClure::Create(k);
  const double weight1 = gmc->weight(error1),
               weight2 = gmc->weight(error2);
  DOUBLES_EQUAL(0.25      , weight1, 1e-8);
  DOUBLES_EQUAL(9.80296e-5, weight2, 1e-8);
}

TEST(NoiseModel, robustFunctionDCS)
{
  const double k = 1.0, error1 = 1.0, error2 = 10.0;
  const mEstimator::DCS::shared_ptr dcs = mEstimator::DCS::Create(k);
  const double weight1 = dcs->weight(error1),
               weight2 = dcs->weight(error2);
  DOUBLES_EQUAL(1.0       , weight1, 1e-8);
  DOUBLES_EQUAL(0.00039211, weight2, 1e-8);
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

/* ************************************************************************* */
#define TEST_GAUSSIAN(gaussian)\
  EQUALITY(info, gaussian->information());\
  EQUALITY(cov, gaussian->covariance());\
  EXPECT(assert_equal(white, gaussian->whiten(e)));\
  EXPECT(assert_equal(e, gaussian->unwhiten(white)));\
  EXPECT_DOUBLES_EQUAL(251, gaussian->distance(e), 1e-9);\
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
  Matrix I = Matrix3::Identity();


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

/* ************************************************************************* */
int main() {  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

/**
 * @file testGeneralSFMFactor.cpp
 * @date Dec 27, 2010
 * @author nikai
 * @brief unit tests for GeneralSFMFactor
 */

#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <CppUnitLite/TestHarness.h>
using namespace boost;

// Magically casts strings like "x3" to a Symbol('x',3) key, see Key.h
#define GTSAM_MAGIC_KEY

#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/TupleValues.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

#include <gtsam/geometry/GeneralCameraT.h>
#include <gtsam/slam/GeneralSFMFactor.h>

using namespace std;
using namespace gtsam;

typedef Cal3BundlerCamera GeneralCamera;
typedef TypedSymbol<GeneralCamera, 'x'> CameraKey;
typedef TypedSymbol<Point3, 'l'> PointKey;
typedef Values<CameraKey> CameraConfig;
typedef Values<PointKey> PointConfig;
typedef TupleValues2<CameraConfig, PointConfig> VisualValues;
typedef GeneralSFMFactor<VisualValues, CameraKey, PointKey> Projection;
typedef NonlinearEquality<VisualValues, CameraKey> CameraConstraint;
typedef NonlinearEquality<VisualValues, PointKey> Point3Constraint;

/* ************************************************************************* */
class Graph: public NonlinearFactorGraph<VisualValues> {
public:
  void addMeasurement(const CameraKey& i, const PointKey& j, const Point2& z, const SharedNoiseModel& model) {
    push_back(boost::make_shared<Projection>(z, model, i, j));
  }

  void addCameraConstraint(int j, const GeneralCamera& p) {
    boost::shared_ptr<CameraConstraint> factor(new CameraConstraint(j, p));
    push_back(factor);
  }

  void addPoint3Constraint(int j, const Point3& p) {
    boost::shared_ptr<Point3Constraint> factor(new Point3Constraint(j, p));
    push_back(factor);
  }

};

double getGaussian()
{
    double S,V1,V2;
    // Use Box-Muller method to create gauss noise from uniform noise
    do
    {
        double U1 = rand() / (double)(RAND_MAX);
        double U2 = rand() / (double)(RAND_MAX);
        V1 = 2 * U1 - 1;           /* V1=[-1,1] */
        V2 = 2 * U2 - 1;           /* V2=[-1,1] */
        S  = V1 * V1 + V2 * V2;
    } while(S>=1);
    return sqrt(-2.0f * (double)log(S) / S) * V1;
}

typedef NonlinearOptimizer<Graph,VisualValues> Optimizer;

const SharedNoiseModel sigma1(noiseModel::Unit::Create(1));

/* ************************************************************************* */
TEST( GeneralSFMFactor, equals )
{
  // Create two identical factors and make sure they're equal
  Vector z = Vector_(2,323.,240.);
  const int cameraFrameNumber=1, landmarkNumber=1;
  const SharedNoiseModel sigma(noiseModel::Unit::Create(1));
  boost::shared_ptr<Projection>
    factor1(new Projection(z, sigma, cameraFrameNumber, landmarkNumber));

  boost::shared_ptr<Projection>
    factor2(new Projection(z, sigma, cameraFrameNumber, landmarkNumber));

  EXPECT(assert_equal(*factor1, *factor2));
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, error ) {
  Point2 z(3.,0.);
  const int cameraFrameNumber=1, landmarkNumber=1;
  const SharedNoiseModel sigma(noiseModel::Unit::Create(1));
  boost::shared_ptr<Projection>
  factor(new Projection(z, sigma, cameraFrameNumber, landmarkNumber));
  // For the following configuration, the factor predicts 320,240
  VisualValues values;
  Rot3 R;
  Point3 t1(0,0,-6);
  Pose3 x1(R,t1);
  values.insert(1, GeneralCamera(x1));
  Point3 l1;  values.insert(1, l1);
  EXPECT(assert_equal(Vector_(2, -3.0, 0.0), factor->unwhitenedError(values)));
}


static const double baseline = 5.0 ;

/* ************************************************************************* */
vector<Point3> genPoint3() {
  const double z = 5;
  vector<Point3> L ;
  L.push_back(Point3 (-1.0,-1.0, z));
  L.push_back(Point3 (-1.0, 1.0, z));
  L.push_back(Point3 ( 1.0, 1.0, z));
  L.push_back(Point3 ( 1.0,-1.0, z));
  L.push_back(Point3 (-1.5,-1.5, 1.5*z));
  L.push_back(Point3 (-1.5, 1.5, 1.5*z));
  L.push_back(Point3 ( 1.5, 1.5, 1.5*z));
  L.push_back(Point3 ( 1.5,-1.5, 1.5*z));
  L.push_back(Point3 (-2.0,-2.0, 2*z));
  L.push_back(Point3 (-2.0, 2.0, 2*z));
  L.push_back(Point3 ( 2.0, 2.0, 2*z));
  L.push_back(Point3 ( 2.0,-2.0, 2*z));
  return L ;
}

vector<GeneralCamera> genCameraDefaultCalibration() {
  vector<GeneralCamera> X ;
  X.push_back(GeneralCamera(Pose3(eye(3),Point3(-baseline/2.0, 0.0, 0.0))));
  X.push_back(GeneralCamera(Pose3(eye(3),Point3( baseline/2.0, 0.0, 0.0))));
  return X ;
}

vector<GeneralCamera> genCameraVariableCalibration() {
  const Cal3Bundler K(500,1e-3,1e-3);
  vector<GeneralCamera> X ;
  X.push_back(GeneralCamera(Pose3(eye(3),Point3(-baseline/2.0, 0.0, 0.0)), K));
  X.push_back(GeneralCamera(Pose3(eye(3),Point3( baseline/2.0, 0.0, 0.0)), K));
  return X ;
}

shared_ptr<Ordering> getOrdering(const vector<GeneralCamera>& X, const vector<Point3>& L) {
  list<Symbol> keys;
  for ( size_t i = 0 ; i < L.size() ; ++i ) keys.push_back(Symbol('l', i)) ;
  for ( size_t i = 0 ; i < X.size() ; ++i ) keys.push_back(Symbol('x', i)) ;
  shared_ptr<Ordering> ordering(new Ordering(keys));
  return ordering ;
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_defaultK ) {

  vector<Point3> L = genPoint3();
  vector<GeneralCamera> X = genCameraDefaultCalibration();

  // add measurement with noise
  shared_ptr<Graph> graph(new Graph());
  for ( size_t j = 0 ; j < X.size() ; ++j) {
    for ( size_t i = 0 ; i < L.size() ; ++i) {
      Point2 pt = X[j].project(L[i]) ;
      graph->addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = X.size()*L.size() ;

  // add initial
  const double noise = baseline*0.1;
  boost::shared_ptr<VisualValues> values(new VisualValues);
  for ( size_t i = 0 ; i < X.size() ; ++i )
    values->insert((int)i, X[i]) ;

  for ( size_t i = 0 ; i < L.size() ; ++i ) {
    Point3 pt(L[i].x()+noise*getGaussian(),
              L[i].y()+noise*getGaussian(),
              L[i].z()+noise*getGaussian());
    values->insert(i, pt) ;
  }

  graph->addCameraConstraint(0, X[0]);

  // Create an ordering of the variables
  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::sharedThis params (
      new NonlinearOptimizationParameters(1e-5, 1e-5, 0.0, 100, 1e-5, 10, NonlinearOptimizationParameters::SILENT));
  Optimizer optimizer(graph, values, ordering, params);
  Optimizer optimizer2 = optimizer.levenbergMarquardt();
  EXPECT(optimizer2.error() < 0.5 * 1e-5 * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_SingleMeasurementError ) {
  vector<Point3> L = genPoint3();
  vector<GeneralCamera> X = genCameraVariableCalibration();
  // add measurement with noise
  shared_ptr<Graph> graph(new Graph());
  for ( size_t j = 0 ; j < X.size() ; ++j) {
    for ( size_t i = 0 ; i < L.size() ; ++i) {
      Point2 pt = X[j].project(L[i]) ;
      graph->addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = X.size()*L.size() ;

  // add initial
  const double noise = baseline*0.1;
  boost::shared_ptr<VisualValues> values(new VisualValues);
  for ( size_t i = 0 ; i < X.size() ; ++i )
    values->insert((int)i, X[i]) ;

  // add noise only to the first landmark
  for ( size_t i = 0 ; i < L.size() ; ++i ) {
    if ( i == 0 ) {
      Point3 pt(L[i].x()+noise*getGaussian(),
                L[i].y()+noise*getGaussian(),
                L[i].z()+noise*getGaussian());
      values->insert(i, pt) ;
    }
    else {
      values->insert(i, L[i]) ;
    }
  }

  graph->addCameraConstraint(0, X[0]);
  const double reproj_error = 1e-5;

  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::sharedThis params (
      new NonlinearOptimizationParameters(1e-5, 1e-5, 0.0, 100, 1e-5, 10, NonlinearOptimizationParameters::SILENT));
  Optimizer optimizer(graph, values, ordering, params);
  Optimizer optimizer2 = optimizer.levenbergMarquardt();
  EXPECT(optimizer2.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_FixCameras ) {

  vector<Point3> L = genPoint3();
  vector<GeneralCamera> X = genCameraVariableCalibration();

  // add measurement with noise
  const double noise = baseline*0.1;
  shared_ptr<Graph> graph(new Graph());
  for ( size_t j = 0 ; j < X.size() ; ++j) {
    for ( size_t i = 0 ; i < L.size() ; ++i) {
      Point2 pt = X[j].project(L[i]) ;
      graph->addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = L.size()*X.size();

  boost::shared_ptr<VisualValues> values(new VisualValues);
  for ( size_t i = 0 ; i < X.size() ; ++i )
    values->insert((int)i, X[i]) ;

  for ( size_t i = 0 ; i < L.size() ; ++i ) {
    Point3 pt(L[i].x()+noise*getGaussian(),
              L[i].y()+noise*getGaussian(),
              L[i].z()+noise*getGaussian());
    //Point3 pt(L[i].x(), L[i].y(), L[i].z());
    values->insert(i, pt) ;
  }

  for ( size_t i = 0 ; i < X.size() ; ++i )
    graph->addCameraConstraint(i, X[i]);

  const double reproj_error = 1e-5 ;

  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::sharedThis params (
      new NonlinearOptimizationParameters(1e-5, 1e-5, 0.0, 100, 1e-3, 10, NonlinearOptimizationParameters::SILENT));
  Optimizer optimizer(graph, values, ordering, params);

  Optimizer optimizer2 = optimizer.levenbergMarquardt();
  EXPECT(optimizer2.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_FixLandmarks ) {

  vector<Point3> L = genPoint3();
  vector<GeneralCamera> X = genCameraVariableCalibration();

  // add measurement with noise
  shared_ptr<Graph> graph(new Graph());
  for ( size_t j = 0 ; j < X.size() ; ++j) {
    for ( size_t i = 0 ; i < L.size() ; ++i) {
      Point2 pt = X[j].project(L[i]) ;
      graph->addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = L.size()*X.size();

  boost::shared_ptr<VisualValues> values(new VisualValues);
  for ( size_t i = 0 ; i < X.size() ; ++i ) {
    const double
      rot_noise = 1e-5, trans_noise = 1e-3,
      focal_noise = 1, distort_noise = 1e-3;
    if ( i == 0 ) {
      values->insert((int)i, X[i]) ;
    }
    else {

      Vector delta = Vector_(9,
          rot_noise, rot_noise, rot_noise, // rotation
          trans_noise, trans_noise, trans_noise, // translation
          focal_noise, distort_noise, distort_noise // f, k1, k2
          ) ;
      values->insert((int)i, X[i].retract(delta)) ;
    }
  }

  for ( size_t i = 0 ; i < L.size() ; ++i ) {
    values->insert(i, L[i]) ;
  }

  // fix X0 and all landmarks, allow only the X[1] to move
  graph->addCameraConstraint(0, X[0]);
  for ( size_t i = 0 ; i < L.size() ; ++i )
    graph->addPoint3Constraint(i, L[i]);

  const double reproj_error = 1e-5 ;

  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::sharedThis params (
      new NonlinearOptimizationParameters(1e-5, 1e-5, 0.0, 100, 1e-3, 10, NonlinearOptimizationParameters::SILENT));
  Optimizer optimizer(graph, values, ordering, params);

  Optimizer optimizer2 = optimizer.levenbergMarquardt();
  EXPECT(optimizer2.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_BA ) {
  vector<Point3> L = genPoint3();
  vector<GeneralCamera> X = genCameraVariableCalibration();

  // add measurement with noise
  shared_ptr<Graph> graph(new Graph());
  for ( size_t j = 0 ; j < X.size() ; ++j) {
    for ( size_t i = 0 ; i < L.size() ; ++i) {
      Point2 pt = X[j].project(L[i]) ;
      graph->addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = X.size()*L.size() ;

  // add initial
  const double noise = baseline*0.1;
  boost::shared_ptr<VisualValues> values(new VisualValues);
  for ( size_t i = 0 ; i < X.size() ; ++i )
    values->insert((int)i, X[i]) ;

  // add noise only to the first landmark
  for ( size_t i = 0 ; i < L.size() ; ++i ) {
    Point3 pt(L[i].x()+noise*getGaussian(),
              L[i].y()+noise*getGaussian(),
              L[i].z()+noise*getGaussian());
    values->insert(i, pt) ;
  }

  graph->addCameraConstraint(0, X[0]);
  const double reproj_error = 1e-5 ;

  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::sharedThis params (
      new NonlinearOptimizationParameters(1e-5, 1e-5, 0.0, 100, 1e-5, 10, NonlinearOptimizationParameters::SILENT));
  Optimizer optimizer(graph, values, ordering, params);

  Optimizer optimizer2 = optimizer.levenbergMarquardt();
  EXPECT(optimizer2.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

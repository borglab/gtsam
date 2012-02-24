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

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/GeneralSFMFactor.h>

using namespace std;
using namespace gtsam;

typedef PinholeCamera<Cal3_S2> GeneralCamera;
typedef GeneralSFMFactor<GeneralCamera, Point3> Projection;
typedef NonlinearEquality<GeneralCamera> CameraConstraint;
typedef NonlinearEquality<Point3> Point3Constraint;

class Graph: public NonlinearFactorGraph {
public:
	void addMeasurement(int i, int j, const Point2& z, const SharedNoiseModel& model) {
		push_back(boost::make_shared<Projection>(z, model, Symbol('x',i), Symbol('l',j)));
	}

	void addCameraConstraint(int j, const GeneralCamera& p) {
		boost::shared_ptr<CameraConstraint> factor(new CameraConstraint(Symbol('x',j), p));
		push_back(factor);
	}

  void addPoint3Constraint(int j, const Point3& p) {
    boost::shared_ptr<Point3Constraint> factor(new Point3Constraint(Symbol('l',j), p));
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

typedef NonlinearOptimizer<Graph> Optimizer;

const SharedNoiseModel sigma1(noiseModel::Unit::Create(1));

/* ************************************************************************* */
TEST( GeneralSFMFactor, equals )
{
	// Create two identical factors and make sure they're equal
	Vector z = Vector_(2,323.,240.);
  const Symbol cameraFrameNumber('x',1), landmarkNumber('l',1);
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
	const SharedNoiseModel sigma(noiseModel::Unit::Create(1));
	boost::shared_ptr<Projection>	factor(new Projection(z, sigma, Symbol('x',1), Symbol('l',1)));
	// For the following configuration, the factor predicts 320,240
	Values values;
	Rot3 R;
	Point3 t1(0,0,-6);
	Pose3 x1(R,t1);
	values.insert(Symbol('x',1), GeneralCamera(x1));
	Point3 l1;  values.insert(Symbol('l',1), l1);
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
  const Cal3_S2 K(640,480,0.01,320,240);
  vector<GeneralCamera> X ;
  X.push_back(GeneralCamera(Pose3(eye(3),Point3(-baseline/2.0, 0.0, 0.0)), K));
  X.push_back(GeneralCamera(Pose3(eye(3),Point3( baseline/2.0, 0.0, 0.0)), K));
  return X ;
}

shared_ptr<Ordering> getOrdering(const vector<GeneralCamera>& X, const vector<Point3>& L) {
  shared_ptr<Ordering> ordering(new Ordering);
  for ( size_t i = 0 ; i < L.size() ; ++i ) ordering->push_back(Symbol('l', i)) ;
  for ( size_t i = 0 ; i < X.size() ; ++i ) ordering->push_back(Symbol('x', i)) ;
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
	boost::shared_ptr<Values> values(new Values);
	for ( size_t i = 0 ; i < X.size() ; ++i )
	  values->insert(Symbol('x',i), X[i]) ;

	for ( size_t i = 0 ; i < L.size() ; ++i ) {
		Point3 pt(L[i].x()+noise*getGaussian(),
		          L[i].y()+noise*getGaussian(),
		          L[i].z()+noise*getGaussian());
		values->insert(Symbol('l',i), pt) ;
	}

	graph->addCameraConstraint(0, X[0]);

	// Create an ordering of the variables
  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::shared_ptr params (
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
  boost::shared_ptr<Values> values(new Values);
  for ( size_t i = 0 ; i < X.size() ; ++i )
    values->insert(Symbol('x',i), X[i]) ;

  // add noise only to the first landmark
  for ( size_t i = 0 ; i < L.size() ; ++i ) {
    if ( i == 0 ) {
      Point3 pt(L[i].x()+noise*getGaussian(),
                L[i].y()+noise*getGaussian(),
                L[i].z()+noise*getGaussian());
      values->insert(Symbol('l',i), pt) ;
    }
    else {
      values->insert(Symbol('l',i), L[i]) ;
    }
  }

  graph->addCameraConstraint(0, X[0]);
  const double reproj_error = 1e-5;

  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::shared_ptr params (
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

  boost::shared_ptr<Values> values(new Values);
  for ( size_t i = 0 ; i < X.size() ; ++i )
    values->insert(Symbol('x',i), X[i]) ;

  for ( size_t i = 0 ; i < L.size() ; ++i ) {
    Point3 pt(L[i].x()+noise*getGaussian(),
              L[i].y()+noise*getGaussian(),
              L[i].z()+noise*getGaussian());
    //Point3 pt(L[i].x(), L[i].y(), L[i].z());
    values->insert(Symbol('l',i), pt) ;
  }

  for ( size_t i = 0 ; i < X.size() ; ++i )
    graph->addCameraConstraint(i, X[i]);

  const double reproj_error = 1e-5 ;

  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::shared_ptr params (
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

  boost::shared_ptr<Values> values(new Values);
  for ( size_t i = 0 ; i < X.size() ; ++i ) {
    const double
      rot_noise = 1e-5,
      trans_noise = 1e-3,
      focal_noise = 1,
      skew_noise = 1e-5;
    if ( i == 0 ) {
      values->insert(Symbol('x',i), X[i]) ;
    }
    else {

      Vector delta = Vector_(11,
          rot_noise, rot_noise, rot_noise, // rotation
          trans_noise, trans_noise, trans_noise, // translation
          focal_noise, focal_noise, // f_x, f_y
          skew_noise, // s
          trans_noise, trans_noise // ux, uy
          ) ;
      values->insert(Symbol('x',i), X[i].retract(delta)) ;
    }
  }

  for ( size_t i = 0 ; i < L.size() ; ++i ) {
    values->insert(Symbol('l',i), L[i]) ;
  }

  // fix X0 and all landmarks, allow only the X[1] to move
  graph->addCameraConstraint(0, X[0]);
  for ( size_t i = 0 ; i < L.size() ; ++i )
    graph->addPoint3Constraint(i, L[i]);

  const double reproj_error = 1e-5 ;

  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::shared_ptr params (
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
  boost::shared_ptr<Values> values(new Values);
  for ( size_t i = 0 ; i < X.size() ; ++i )
    values->insert(Symbol('x',i), X[i]) ;

  // add noise only to the first landmark
  for ( size_t i = 0 ; i < L.size() ; ++i ) {
    Point3 pt(L[i].x()+noise*getGaussian(),
              L[i].y()+noise*getGaussian(),
              L[i].z()+noise*getGaussian());
    values->insert(Symbol('l',i), pt) ;
  }

  graph->addCameraConstraint(0, X[0]);
  const double reproj_error = 1e-5 ;

  shared_ptr<Ordering> ordering = getOrdering(X,L);
  NonlinearOptimizationParameters::shared_ptr params (
      new NonlinearOptimizationParameters(1e-2, 1e-2, 0.0, 100, 1e-5, 10, NonlinearOptimizationParameters::SILENT));
  Optimizer optimizer(graph, values, ordering, params);

  Optimizer optimizer2 = optimizer.levenbergMarquardt();
  EXPECT(optimizer2.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

/*
 * testGeneralSFMFactor.cpp
 *
 *   Created on: Dec 27, 2010
 *       Author: nikai
 *  Description: unit tests for GeneralSFMFactor
 */

#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <CppUnitLite/TestHarness.h>
using namespace boost;

#define GTSAM_MAGIC_KEY

#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/TupleValues-inl.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

#include <gtsam/geometry/GeneralCameraT.h>
#include <gtsam/slam/GeneralSFMFactor.h>

using namespace std;
using namespace gtsam;

typedef Cal3_S2Camera GeneralCamera;
typedef TypedSymbol<GeneralCamera, 'x'> CameraKey;
typedef TypedSymbol<Point3, 'l'> PointKey;
typedef LieValues<CameraKey> CameraConfig;
typedef LieValues<PointKey> PointConfig;
typedef TupleValues2<CameraConfig, PointConfig> Values;
typedef GeneralSFMFactor<Values, CameraKey, PointKey> Projection;
typedef NonlinearEquality<Values, CameraKey> CameraConstraint;

class Graph: public NonlinearFactorGraph<Values> {
public:
	void addMeasurement(const CameraKey& i, const PointKey& j, const Point2& z, const SharedGaussian& model) {
		push_back(boost::make_shared<Projection>(z, model, i, j));
	}

	void addCameraConstraint(int j, const GeneralCamera& p) {
		boost::shared_ptr<CameraConstraint> factor(new CameraConstraint(j, p));
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

typedef NonlinearOptimizer<Graph,Values> Optimizer;

// make cube
static Point3
	x000(-1, -1, -1), x001(-1, -1, +1), x010(-1, +1, -1), x011(-1, +1, +1),
	x100(-1, -1, -1), x101(-1, -1, +1), x110(-1, +1, -1), x111(-1, +1, +1);


/* ************************************************************************* */
TEST( GeneralSFMFactor, equals )
{
	// Create two identical factors and make sure they're equal
	Vector z = Vector_(2,323.,240.);
	const int cameraFrameNumber=1, landmarkNumber=1;
	const SharedGaussian sigma(noiseModel::Unit::Create(1));
	boost::shared_ptr<Projection>
	  factor1(new Projection(z, sigma, cameraFrameNumber, landmarkNumber));

	boost::shared_ptr<Projection>
		factor2(new Projection(z, sigma, cameraFrameNumber, landmarkNumber));

	CHECK(assert_equal(*factor1, *factor2));
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, error ) {

	Point2 z(3.,0.);
	const int cameraFrameNumber=1, landmarkNumber=1;
	const SharedGaussian sigma(noiseModel::Unit::Create(1));

	boost::shared_ptr<Projection>
	factor(new Projection(z, sigma, cameraFrameNumber, landmarkNumber));

	// For the following configuration, the factor predicts 320,240
	Values values;
	Rot3 R;
	Point3 t1(0,0,-6);
	Pose3 x1(R,t1);
	values.insert(1, GeneralCamera(x1));
	Point3 l1;  values.insert(1, l1);
	CHECK(assert_equal(Vector_(2, -3.0, 0.0), factor->unwhitenedError(values)));
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize ) {
	const SharedGaussian sigma1(noiseModel::Unit::Create(1));
	const double z = 5;
	vector<Point3> L ;
	L.push_back(Point3 (-1.0,-1.0, z));
	L.push_back(Point3 (-1.0, 1.0, z));
	L.push_back(Point3 ( 1.0, 1.0, z));
	L.push_back(Point3 ( 1.0,-1.0, z));
	L.push_back(Point3 (-1.0,-1.0, 2*z));
	L.push_back(Point3 (-1.0, 1.0, 2*z));
	L.push_back(Point3 ( 1.0, 1.0, 2*z));
	L.push_back(Point3 ( 1.0,-1.0, 2*z));

	vector<GeneralCamera> X ;
	X.push_back(GeneralCamera(Pose3()));
	X.push_back(GeneralCamera(Pose3(eye(3),Point3(0.0,0.0,-z))));

	// add measurement with noise
	shared_ptr<Graph> graph(new Graph());
	for ( size_t j = 0 ; j < X.size() ; ++j) {
		for ( size_t i = 0 ; i < L.size() ; ++i) {
			Point2 pt = X[j].project(L[i]) ;
			//Point2 q(pt.x()+0.1*getGaussian(), pt.y()+0.1*getGaussian()) ;
			graph->addMeasurement(j, i, pt, sigma1);
		}
	}

	// add initial
	const double noise = 1.0;
	boost::shared_ptr<Values> values(new Values);
	for ( int i = 0 ; i < X.size() ; ++i )
	  values->insert(i, X[i]) ;

	for ( size_t i = 0 ; i < L.size() ; ++i ) {
		Point3 pt(L[i].x()+noise*getGaussian(),
		          L[i].y()+noise*getGaussian(),
		          L[i].z()+noise*getGaussian());

		//if (i == 0) pt = Point3(pt.x()+10, pt.y(), pt.z()) ;
		values->insert(i, pt) ;
	}

	graph->addCameraConstraint(0, X[0]);

	// Create an ordering of the variables
	list<Symbol> keys;
	for ( size_t i = 0 ; i < L.size() ; ++i ) keys.push_back(Symbol('l', i)) ;
	for ( size_t i = 0 ; i < X.size() ; ++i ) keys.push_back(Symbol('x', i)) ;
	shared_ptr<Ordering> ordering(new Ordering(keys));

  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-7, 1e-7);
	Optimizer optimizer(graph, values, ordering, params);
	cout << "before optimization, error is " << optimizer.error() << endl;
	Optimizer optimizer2 = optimizer.levenbergMarquardt();
	cout << "after optimization, error is " << optimizer2.error() << endl;
	CHECK(optimizer2.error() < 1e-1);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

/**
 * @file    smallExample.cpp
 * @brief   Create small example with two poses and one landmark
 * @brief   smallExample
 * @author  Carlos Nieto
 * @author  Frank dellaert
 */

#include <iostream>
#include <string>

using namespace std;

#include "Ordering.h"
#include "Matrix.h"
#include "NonlinearFactor.h"
#include "smallExample.h"
#include "Point2Prior.h"
#include "Simulated2DOdometry.h"
#include "Simulated2DMeasurement.h"
#include "simulated2D.h"

// template definitions
#include "FactorGraph-inl.h"
#include "NonlinearFactorGraph-inl.h"

namespace gtsam {

typedef boost::shared_ptr<NonlinearFactor<VectorConfig> > shared;

/* ************************************************************************* */
boost::shared_ptr<const ExampleNonlinearFactorGraph> sharedNonlinearFactorGraph() {
	// Create
	boost::shared_ptr<ExampleNonlinearFactorGraph> nlfg(new ExampleNonlinearFactorGraph);

	// prior on x1
	double sigma1=0.1;
	Vector mu(2); mu(0) = 0 ; mu(1) = 0;
	shared f1(new Point2Prior(mu, sigma1, "x1"));
	nlfg->push_back(f1);

	// odometry between x1 and x2
	double sigma2=0.1;
	Vector z2(2); z2(0) = 1.5 ; z2(1) = 0;
	shared f2(new Simulated2DOdometry(z2, sigma2, "x1", "x2"));
	nlfg->push_back(f2);

	// measurement between x1 and l1
	double sigma3=0.2;
	Vector z3(2); z3(0) = 0. ; z3(1) = -1.;
	shared f3(new Simulated2DMeasurement(z3, sigma3, "x1", "l1"));
	nlfg->push_back(f3);

	// measurement between x2 and l1
	double sigma4=0.2;
	Vector z4(2); z4(0)= -1.5 ; z4(1) = -1.;
	shared f4(new Simulated2DMeasurement(z4, sigma4, "x2", "l1"));
	nlfg->push_back(f4);

	return nlfg;
}

/* ************************************************************************* */
ExampleNonlinearFactorGraph createNonlinearFactorGraph() {
	return *sharedNonlinearFactorGraph();
}

/* ************************************************************************* */
VectorConfig createConfig()
{
    Vector v_x1(2); v_x1(0) = 0.;  v_x1(1) = 0.;
    Vector v_x2(2); v_x2(0) = 1.5; v_x2(1) = 0.;
    Vector v_l1(2); v_l1(0) = 0.;  v_l1(1) = -1.;
    VectorConfig c;
    c.insert("x1", v_x1);
    c.insert("x2", v_x2);
    c.insert("l1", v_l1);
    return c;
}

/* ************************************************************************* */
boost::shared_ptr<const VectorConfig> sharedNoisyConfig()
{
    Vector v_x1(2); v_x1(0) = 0.1;  v_x1(1) = 0.1;
    Vector v_x2(2); v_x2(0) = 1.4;  v_x2(1) = 0.2;
    Vector v_l1(2); v_l1(0) = 0.1;  v_l1(1) = -1.1;
    boost::shared_ptr<VectorConfig> c(new VectorConfig);
    c->insert("x1", v_x1);
    c->insert("x2", v_x2);
    c->insert("l1", v_l1);
    return c;
}

/* ************************************************************************* */
VectorConfig createNoisyConfig() {
	return *sharedNoisyConfig();
}

/* ************************************************************************* */
VectorConfig createCorrectDelta() {
  Vector v_x1(2); v_x1(0) = -0.1;  v_x1(1) = -0.1;
  Vector v_x2(2); v_x2(0) =  0.1;  v_x2(1) = -0.2;
  Vector v_l1(2); v_l1(0) = -0.1;  v_l1(1) =  0.1;
  VectorConfig c;
  c.insert("x1", v_x1);
  c.insert("x2", v_x2);
  c.insert("l1", v_l1);
  return c;
}

/* ************************************************************************* */
VectorConfig createZeroDelta() {
  Vector v_x1(2); v_x1(0) = 0;  v_x1(1) = 0;
  Vector v_x2(2); v_x2(0) = 0;  v_x2(1) = 0;
  Vector v_l1(2); v_l1(0) = 0;  v_l1(1) = 0;
  VectorConfig c;
  c.insert("x1", v_x1);
  c.insert("x2", v_x2);
  c.insert("l1", v_l1);
  return c;
}

/* ************************************************************************* */
GaussianFactorGraph createGaussianFactorGraph()
{
  VectorConfig c = createNoisyConfig();
  
  // Create
  GaussianFactorGraph fg;

  double sigma1 = 0.1;

  // prior on x1
  Matrix A11(2,2);
  A11(0,0) = 1; A11(0,1) =  0;
  A11(1,0) =  0; A11(1,1) = 1;

  Vector b = - c["x1"];

  GaussianFactor::shared_ptr f1(new GaussianFactor("x1", A11, b, sigma1));
  fg.push_back(f1);

  // odometry between x1 and x2
  double sigma2 = 0.1;
  Matrix A21(2,2);
  A21(0,0) = -1 ; A21(0,1) =   0;
  A21(1,0) =   0 ; A21(1,1) = -1;

  Matrix A22(2,2);
  A22(0,0) = 1 ; A22(0,1) =  0;
  A22(1,0) =  0 ; A22(1,1) = 1;

  // Vector b(2);
  b(0) = 0.2 ; b(1) = -0.1;

  GaussianFactor::shared_ptr f2(new GaussianFactor("x1", A21,  "x2", A22, b, sigma2));
  fg.push_back(f2);

  // measurement between x1 and l1
  double sigma3 = 0.2;
  Matrix A31(2,2);
  A31(0,0) = -1; A31(0,1) =  0;
  A31(1,0) =  0; A31(1,1) = -1;

  Matrix A32(2,2);
  A32(0,0) = 1 ; A32(0,1) = 0;
  A32(1,0) = 0 ; A32(1,1) = 1;

  b(0) = 0 ; b(1) = 0.2;

  GaussianFactor::shared_ptr f3(new GaussianFactor("x1", A31, "l1", A32, b, sigma3));
  fg.push_back(f3);

  // measurement between x2 and l1
  double sigma4 = 0.2;
  Matrix A41(2,2);
  A41(0,0) = -1 ; A41(0,1) =  0;
  A41(1,0) =  0 ; A41(1,1) = -1;

  Matrix A42(2,2);
  A42(0,0) = 1 ; A42(0,1) = 0;
  A42(1,0) = 0 ; A42(1,1) = 1;

  b(0)= -0.2 ; b(1) = 0.3;

  GaussianFactor::shared_ptr f4(new GaussianFactor("x2", A41, "l1", A42, b, sigma4));
  fg.push_back(f4);

  return fg;
}

/* ************************************************************************* */
/** create small Chordal Bayes Net x <- y
 * x y d
 * 1 1 9
 *   1 5
 */
GaussianBayesNet createSmallGaussianBayesNet()
{
  Matrix R11 = Matrix_(1,1,1.0), S12 = Matrix_(1,1,1.0);
  Matrix                          R22 = Matrix_(1,1,1.0);
  Vector d1(1), d2(1);
  d1(0) = 9; d2(0) = 5;
  Vector tau(1); tau(0) = 1.0;
  
  // define nodes and specify in reverse topological sort (i.e. parents last)
  GaussianConditional::shared_ptr
    Px_y(new GaussianConditional("x",d1,R11,"y",S12,tau)),
    Py(new GaussianConditional("y",d2,R22,tau));
  GaussianBayesNet cbn;
  cbn.push_back(Px_y);
  cbn.push_back(Py);

  return cbn;
}

/* ************************************************************************* */
// Some nonlinear functions to optimize
/* ************************************************************************* */
namespace smallOptimize {
  Vector h(const Vector& v) {
    double x = v(0);
    return Vector_(2,cos(x),sin(x));
  };
  Matrix H(const Vector& v) {
    double x = v(0);
    return Matrix_(2,1,-sin(x),cos(x));
  };
}

/* ************************************************************************* */
boost::shared_ptr<const ExampleNonlinearFactorGraph> sharedReallyNonlinearFactorGraph()
{
	boost::shared_ptr<ExampleNonlinearFactorGraph> fg(new ExampleNonlinearFactorGraph);
  Vector z = Vector_(2,1.0,0.0);
  double sigma = 0.1;
  boost::shared_ptr<NonlinearFactor1> 
    factor(new NonlinearFactor1(z,sigma,&smallOptimize::h,"x",&smallOptimize::H));
  fg->push_back(factor);
  return fg;
}

ExampleNonlinearFactorGraph createReallyNonlinearFactorGraph() {
	return *sharedReallyNonlinearFactorGraph();
}

/* ************************************************************************* */
GaussianFactorGraph createSmoother(int T) {

	// noise on measurements and odometry, respectively
	double sigma1 = 1, sigma2 = 1;

	// Create
	ExampleNonlinearFactorGraph nlfg;
	VectorConfig poses;

	// prior on x1
	Vector x1 = Vector_(2,1.0,0.0);
	string key1 = symbol('x', 1);
	shared prior(new Point2Prior(x1, sigma1, key1));
	nlfg.push_back(prior);
	poses.insert(key1, x1);

	for (int t = 2; t <= T; t++) {
		// odometry between x_t and x_{t-1}
		Vector odo = Vector_(2, 1.0, 0.0);
		string key = symbol('x', t);
		shared odometry(new Simulated2DOdometry(odo, sigma2, symbol('x', t - 1), key));
		nlfg.push_back(odometry);

		// measurement on x_t is like perfect GPS
		Vector xt = Vector_(2, (double)t, 0.0);
		shared measurement(new Point2Prior(xt, sigma1, key));
		nlfg.push_back(measurement);

		// initial estimate
		poses.insert(key, xt);
	}

	GaussianFactorGraph lfg = nlfg.linearize(poses);
	return lfg;
}

/* ************************************************************************* */
GaussianFactorGraph createSimpleConstraintGraph() {
	// create unary factor
	// prior on "x", mean = [1,-1], sigma=0.1
	double sigma = 0.1;
	Matrix Ax = eye(2);
	Vector b1(2);
	b1(0) = 1.0;
	b1(1) = -1.0;
	GaussianFactor::shared_ptr f1(new GaussianFactor("x", Ax, b1, sigma));

	// create binary constraint factor
	// between "x" and "y", that is going to be the only factor on "y"
	// |1 0||x_1| + |-1  0||y_1| = |0|
	// |0 1||x_2|   | 0 -1||y_2|   |0|
	Matrix Ax1 = eye(2);
	Matrix Ay1 = eye(2) * -1;
	Vector b2 = Vector_(2, 0.0, 0.0);
	GaussianFactor::shared_ptr f2(
			new GaussianFactor("x", Ax1, "y", Ay1, b2, 0.0));

	// construct the graph
	GaussianFactorGraph fg;
	fg.push_back(f1);
	fg.push_back(f2);

	return fg;
}

/* ************************************************************************* */
VectorConfig createSimpleConstraintConfig() {
	VectorConfig config;
	Vector v = Vector_(2, 1.0, -1.0);
	config.insert("x", v);
	config.insert("y", v);
	return config;
}

/* ************************************************************************* */
GaussianFactorGraph createSingleConstraintGraph() {
	// create unary factor
	// prior on "x", mean = [1,-1], sigma=0.1
	double sigma = 0.1;
	Matrix Ax = eye(2);
	Vector b1(2);
	b1(0) = 1.0;
	b1(1) = -1.0;
	GaussianFactor::shared_ptr f1(new GaussianFactor("x", Ax, b1, sigma));

	// create binary constraint factor
	// between "x" and "y", that is going to be the only factor on "y"
	// |1 2||x_1| + |10 0||y_1| = |1|
	// |2 1||x_2|   |0 10||y_2|   |2|
	Matrix Ax1(2, 2);
	Ax1(0, 0) = 1.0; Ax1(0, 1) = 2.0;
	Ax1(1, 0) = 2.0; Ax1(1, 1) = 1.0;
	Matrix Ay1 = eye(2) * 10;
	Vector b2 = Vector_(2, 1.0, 2.0);
	GaussianFactor::shared_ptr f2(
			new GaussianFactor("x", Ax1, "y", Ay1, b2, 0.0));

	// construct the graph
	GaussianFactorGraph fg;
	fg.push_back(f1);
	fg.push_back(f2);

	return fg;
}

/* ************************************************************************* */
VectorConfig createSingleConstraintConfig() {
	VectorConfig config;
	config.insert("x", Vector_(2, 1.0, -1.0));
	config.insert("y", Vector_(2, 0.2,  0.1));
	return config;
}

/* ************************************************************************* */
GaussianFactorGraph createMultiConstraintGraph() {
	// unary factor 1
	double sigma = 0.1;
	Matrix A = eye(2);
	Vector b = Vector_(2, -2.0, 2.0);
	GaussianFactor::shared_ptr lf1(new GaussianFactor("x", A, b, sigma));

	// constraint 1
	Matrix A11(2,2);
	A11(0,0) = 1.0 ; A11(0,1) = 2.0;
	A11(1,0) = 2.0 ; A11(1,1) = 1.0;

	Matrix A12(2,2);
	A12(0,0) = 10.0 ; A12(0,1) = 0.0;
	A12(1,0) = 0.0 ; A12(1,1) = 10.0;

	Vector b1(2);
	b1(0) = 1.0; b1(1) = 2.0;
	GaussianFactor::shared_ptr lc1(new GaussianFactor("x", A11, "y", A12, b1, 0.0));

	// constraint 2
	Matrix A21(2,2);
	A21(0,0) =  3.0 ; A21(0,1) =  4.0;
	A21(1,0) = -1.0 ; A21(1,1) = -2.0;

	Matrix A22(2,2);
	A22(0,0) = 1.0 ; A22(0,1) = 1.0;
	A22(1,0) = 1.0 ; A22(1,1) = 2.0;

	Vector b2(2);
	b2(0) = 3.0; b2(1) = 4.0;
	GaussianFactor::shared_ptr lc2(new GaussianFactor("x", A21, "z", A22, b2, 0.0));

	// construct the graph
	GaussianFactorGraph fg;
	fg.push_back(lf1);
	fg.push_back(lc1);
	fg.push_back(lc2);

	return fg;
}

/* ************************************************************************* */
VectorConfig createMultiConstraintConfig() {
	VectorConfig config;
	config.insert("x", Vector_(2, -2.0, 2.0));
	config.insert("y", Vector_(2, -0.1, 0.4));
	config.insert("z", Vector_(2, -4.0, 5.0));
	return config;
}

/* ************************************************************************* */
//GaussianFactorGraph createConstrainedGaussianFactorGraph()
//{
//	GaussianFactorGraph graph;
//
//	// add an equality factor
//	Vector v1(2); v1(0)=1.;v1(1)=2.;
//	GaussianFactor::shared_ptr f1(new GaussianFactor(v1, "x0"));
//	graph.push_back_eq(f1);
//
//	// add a normal linear factor
//	Matrix A21 = -1 * eye(2);
//
//	Matrix A22 = eye(2);
//
//	Vector b(2);
//	b(0) = 2 ; b(1) = 3;
//
//	double sigma = 0.1;
//	GaussianFactor::shared_ptr f2(new GaussianFactor("x0", A21/sigma,  "x1", A22/sigma, b/sigma));
//	graph.push_back(f2);
//	return graph;
//}

/* ************************************************************************* */
//	ConstrainedNonlinearFactorGraph<NonlinearFactor<VectorConfig> , VectorConfig> createConstrainedNonlinearFactorGraph() {
//		ConstrainedNonlinearFactorGraph<NonlinearFactor<VectorConfig> , VectorConfig> graph;
//		VectorConfig c = createConstrainedConfig();
//
//		// equality constraint for initial pose
//		GaussianFactor::shared_ptr f1(new GaussianFactor(c["x0"], "x0"));
//		graph.push_back_eq(f1);
//
//		// odometry between x0 and x1
//		double sigma = 0.1;
//		shared f2(new Simulated2DOdometry(c["x1"] - c["x0"], sigma, "x0", "x1"));
//		graph.push_back(f2); // TODO
//		return graph;
//	}

/* ************************************************************************* */
//VectorConfig createConstrainedConfig()
//{
//	VectorConfig config;
//
//	Vector x0(2); x0(0)=1.0; x0(1)=2.0;
//	config.insert("x0", x0);
//
//	Vector x1(2); x1(0)=3.0; x1(1)=5.0;
//	config.insert("x1", x1);
//
//	return config;
//}

/* ************************************************************************* */
//VectorConfig createConstrainedLinConfig()
//{
//	VectorConfig config;
//
//	Vector x0(2); x0(0)=1.0; x0(1)=2.0; // value doesn't actually matter
//	config.insert("x0", x0);
//
//	Vector x1(2); x1(0)=2.3; x1(1)=5.3;
//	config.insert("x1", x1);
//
//	return config;
//}

/* ************************************************************************* */
//VectorConfig createConstrainedCorrectDelta()
//{
//	VectorConfig config;
//
//	Vector x0(2); x0(0)=0.; x0(1)=0.;
//	config.insert("x0", x0);
//
//	Vector x1(2); x1(0)= 0.7; x1(1)= -0.3;
//	config.insert("x1", x1);
//
//	return config;
//}

/* ************************************************************************* */
//ConstrainedGaussianBayesNet createConstrainedGaussianBayesNet()
//{
//	ConstrainedGaussianBayesNet cbn;
//	VectorConfig c = createConstrainedConfig();
//
//	// add regular conditional gaussian - no parent
//	Matrix R = eye(2);
//	Vector d = c["x1"];
//	double sigma = 0.1;
//	GaussianConditional::shared_ptr f1(new GaussianConditional(d/sigma, R/sigma));
//	cbn.insert("x1", f1);
//
//	// add a delta function to the cbn
//	ConstrainedGaussianConditional::shared_ptr f2(new ConstrainedGaussianConditional); //(c["x0"], "x0"));
//	cbn.insert_df("x0", f2);
//
//	return cbn;
//}

} // namespace gtsam

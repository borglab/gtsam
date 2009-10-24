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
#include "ConstrainedLinearFactorGraph.h"
#include "smallExample.h"
#include "Point2Prior.h"
#include "Simulated2DOdometry.h"
#include "Simulated2DMeasurement.h"
#include "simulated2D.h"
#include "FactorGraph-inl.h"

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
LinearFactorGraph createLinearFactorGraph()
{
  VectorConfig c = createNoisyConfig();
  
  // Create
  LinearFactorGraph fg;

  // prior on x1
  Matrix A11(2,2);
  A11(0,0) = 10; A11(0,1) =  0;
  A11(1,0) =  0; A11(1,1) = 10;

  Vector b = - c["x1"]/0.1;

  LinearFactor::shared_ptr f1(new LinearFactor("x1", A11, b));
  fg.push_back(f1);

  // odometry between x1 and x2
  Matrix A21(2,2);
  A21(0,0) = -10 ; A21(0,1) =   0;
  A21(1,0) =   0 ; A21(1,1) = -10;

  Matrix A22(2,2);
  A22(0,0) = 10 ; A22(0,1) =  0;
  A22(1,0) =  0 ; A22(1,1) = 10;

  // Vector b(2);
  b(0) = 2 ; b(1) = -1;

  LinearFactor::shared_ptr f2(new LinearFactor("x1", A21,  "x2", A22, b));
  fg.push_back(f2);

  // measurement between x1 and l1
  Matrix A31(2,2);
  A31(0,0) = -5; A31(0,1) =  0;
  A31(1,0) =  0; A31(1,1) = -5;

  Matrix A32(2,2);
  A32(0,0) = 5 ; A32(0,1) = 0;
  A32(1,0) = 0 ; A32(1,1) = 5;

  b(0) = 0 ; b(1) = 1;

  LinearFactor::shared_ptr f3(new LinearFactor("x1", A31, "l1", A32, b));
  fg.push_back(f3);

  // measurement between x2 and l1
  Matrix A41(2,2);
  A41(0,0) = -5 ; A41(0,1) =  0;
  A41(1,0) =  0 ; A41(1,1) = -5;

  Matrix A42(2,2);
  A42(0,0) = 5 ; A42(0,1) = 0;
  A42(1,0) = 0 ; A42(1,1) = 5;

  b(0)= -1 ; b(1) = 1.5;

  LinearFactor::shared_ptr f4(new LinearFactor("x2", A41, "l1", A42, b));
  fg.push_back(f4);

  return fg;
}

/* ************************************************************************* */
/** create small Chordal Bayes Net x <- y
 * x y d
 * 1 1 9
 *   1 5
 */
ChordalBayesNet createSmallChordalBayesNet()
{
  Matrix R11 = Matrix_(1,1,1.0), S12 = Matrix_(1,1,1.0);
  Matrix                          R22 = Matrix_(1,1,1.0);
  Vector d1(1), d2(1);
  d1(0) = 9; d2(0) = 5;
  
  // define nodes and specify in reverse topological sort (i.e. parents last)
  ConditionalGaussian::shared_ptr
    x(new ConditionalGaussian(d1,R11,"y",S12)),
    y(new ConditionalGaussian(d2,R22));
  ChordalBayesNet cbn;
  cbn.insert("x",x);
  cbn.insert("y",y);

  return cbn;
}

/* ************************************************************************* */
// Some nonlinear functions to optimize
/* ************************************************************************* */
namespace optimize {
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
    factor(new NonlinearFactor1(z,sigma,&optimize::h,"x",&optimize::H));
  fg->push_back(factor);
  return fg;
}

ExampleNonlinearFactorGraph createReallyNonlinearFactorGraph() {
	return *sharedReallyNonlinearFactorGraph();
}

/* ************************************************************************* */
ConstrainedLinearFactorGraph createSingleConstraintGraph() {
	// create unary factor
	// prior on "x", mean = [1,-1], sigma=0.1
	double sigma = 0.1;
	Matrix Ax = eye(2) / sigma;
	Vector b1(2);
	b1(0) = 1.0;
	b1(1) = -1.0;
	LinearFactor::shared_ptr f1(new LinearFactor("x", Ax, b1 / sigma));

	// create binary constraint factor
	// between "x" and "y", that is going to be the only factor on "y"
	// |1 2||x_1| + |10 0||y_1| = |1|
	// |2 1||x_2|   |0 10||y_2|   |2|
	Matrix Ax1(2, 2);
	Ax1(0, 0) = 1.0; Ax1(0, 1) = 2.0;
	Ax1(1, 0) = 2.0; Ax1(1, 1) = 1.0;
	Matrix Ay1 = eye(2) * 10;
	Vector b2 = Vector_(2, 1.0, 2.0);
	LinearConstraint::shared_ptr f2(
			new LinearConstraint("x", Ax1, "y", Ay1, b2));

	// construct the graph
	ConstrainedLinearFactorGraph fg;
	fg.push_back(f1);
	fg.push_back_constraint(f2);

	return fg;
}

/* ************************************************************************* */
ConstrainedLinearFactorGraph createMultiConstraintGraph() {
	// unary factor 1
	double sigma = 0.1;
	Matrix A = eye(2) / sigma;
	Vector b = Vector_(2, -2.0, 2.0)/sigma;
	LinearFactor::shared_ptr lf1(new LinearFactor("x", A, b));

	// constraint 1
	Matrix A11(2,2);
	A11(0,0) = 1.0 ; A11(0,1) = 2.0;
	A11(1,0) = 2.0 ; A11(1,1) = 1.0;

	Matrix A12(2,2);
	A12(0,0) = 10.0 ; A12(0,1) = 0.0;
	A12(1,0) = 0.0 ; A12(1,1) = 10.0;

	Vector b1(2);
	b1(0) = 1.0; b1(1) = 2.0;
	LinearConstraint::shared_ptr lc1(new LinearConstraint("x", A11, "y", A12, b1));

	// constraint 2
	Matrix A21(2,2);
	A21(0,0) =  3.0 ; A21(0,1) =  4.0;
	A21(1,0) = -1.0 ; A21(1,1) = -2.0;

	Matrix A22(2,2);
	A22(0,0) = 1.0 ; A22(0,1) = 1.0;
	A22(1,0) = 1.0 ; A22(1,1) = 2.0;

	Vector b2(2);
	b2(0) = 3.0; b2(1) = 4.0;
	LinearConstraint::shared_ptr lc2(new LinearConstraint("x", A21, "z", A22, b2));

	// construct the graph
	ConstrainedLinearFactorGraph fg;
	fg.push_back(lf1);
	fg.push_back_constraint(lc1);
	fg.push_back_constraint(lc2);

	return fg;
}

/* ************************************************************************* */
//ConstrainedLinearFactorGraph createConstrainedLinearFactorGraph()
//{
//	ConstrainedLinearFactorGraph graph;
//
//	// add an equality factor
//	Vector v1(2); v1(0)=1.;v1(1)=2.;
//	LinearConstraint::shared_ptr f1(new LinearConstraint(v1, "x0"));
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
//	LinearFactor::shared_ptr f2(new LinearFactor("x0", A21/sigma,  "x1", A22/sigma, b/sigma));
//	graph.push_back(f2);
//	return graph;
//}

/* ************************************************************************* */
//	ConstrainedNonlinearFactorGraph<NonlinearFactor<VectorConfig> , VectorConfig> createConstrainedNonlinearFactorGraph() {
//		ConstrainedNonlinearFactorGraph<NonlinearFactor<VectorConfig> , VectorConfig> graph;
//		VectorConfig c = createConstrainedConfig();
//
//		// equality constraint for initial pose
//		LinearConstraint::shared_ptr f1(new LinearConstraint(c["x0"], "x0"));
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
//ConstrainedChordalBayesNet createConstrainedChordalBayesNet()
//{
//	ConstrainedChordalBayesNet cbn;
//	VectorConfig c = createConstrainedConfig();
//
//	// add regular conditional gaussian - no parent
//	Matrix R = eye(2);
//	Vector d = c["x1"];
//	double sigma = 0.1;
//	ConditionalGaussian::shared_ptr f1(new ConditionalGaussian(d/sigma, R/sigma));
//	cbn.insert("x1", f1);
//
//	// add a delta function to the cbn
//	ConstrainedConditionalGaussian::shared_ptr f2(new ConstrainedConditionalGaussian); //(c["x0"], "x0"));
//	cbn.insert_df("x0", f2);
//
//	return cbn;
//}

} // namespace gtsam

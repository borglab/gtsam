/**
 * @file    smallExample.cpp
 * @brief   Create small example with two poses and one landmark
 * @brief   smallExample
 * @author  Carlos Nieto
 * @author  Frank dellaert
 */

#include <iostream>
#include <string>
#include <boost/optional.hpp>

using namespace std;

#define GTSAM_MAGIC_KEY

#include "Ordering.h"
#include "Matrix.h"
#include "NonlinearFactor.h"
#include "smallExample.h"

// template definitions
#include "FactorGraph-inl.h"
#include "NonlinearFactorGraph-inl.h"

namespace gtsam {
namespace example {

	typedef boost::shared_ptr<NonlinearFactor<Config> > shared;

	static SharedDiagonal sigma1_0 = noiseModel::Isotropic::Sigma(2,1.0);
	static SharedDiagonal sigma0_1 = noiseModel::Isotropic::Sigma(2,0.1);
	static SharedDiagonal sigma0_2 = noiseModel::Isotropic::Sigma(2,0.2);
	static SharedDiagonal constraintModel = noiseModel::Constrained::All(2);

	/* ************************************************************************* */
	boost::shared_ptr<const Graph> sharedNonlinearFactorGraph() {
		// Create
		boost::shared_ptr<Graph> nlfg(
				new Graph);

		// prior on x1
		Point2 mu;
		shared f1(new simulated2D::Prior(mu, sigma0_1, 1));
		nlfg->push_back(f1);

		// odometry between x1 and x2
		Point2 z2(1.5, 0);
		shared f2(new simulated2D::Odometry(z2, sigma0_1, 1, 2));
		nlfg->push_back(f2);

		// measurement between x1 and l1
		Point2 z3(0, -1);
		shared f3(new simulated2D::Measurement(z3, sigma0_2, 1, 1));
		nlfg->push_back(f3);

		// measurement between x2 and l1
		Point2 z4(-1.5, -1.);
		shared f4(new simulated2D::Measurement(z4, sigma0_2, 2, 1));
		nlfg->push_back(f4);

		return nlfg;
	}

	/* ************************************************************************* */
	Graph createNonlinearFactorGraph() {
		return *sharedNonlinearFactorGraph();
	}

	/* ************************************************************************* */
	Config createConfig() {
		Config c;
		c.insert(simulated2D::PoseKey(1), Point2(0.0, 0.0));
		c.insert(simulated2D::PoseKey(2), Point2(1.5, 0.0));
		c.insert(simulated2D::PointKey(1), Point2(0.0, -1.0));
		return c;
	}

	/* ************************************************************************* */
	VectorConfig createVectorConfig() {
		VectorConfig c;
		c.insert("l1", Vector_(2, 0.0, -1.0));
		c.insert("x1", Vector_(2, 0.0, 0.0));
		c.insert("x2", Vector_(2, 1.5, 0.0));
		return c;
	}

	/* ************************************************************************* */
	boost::shared_ptr<const Config> sharedNoisyConfig() {
		boost::shared_ptr<Config> c(new Config);
		c->insert(simulated2D::PoseKey(1), Point2(0.1, 0.1));
		c->insert(simulated2D::PoseKey(2), Point2(1.4, 0.2));
		c->insert(simulated2D::PointKey(1), Point2(0.1, -1.1));
		return c;
	}

	/* ************************************************************************* */
	Config createNoisyConfig() {
		return *sharedNoisyConfig();
	}

	/* ************************************************************************* */
	VectorConfig createCorrectDelta() {
		VectorConfig c;
		c.insert("l1", Vector_(2, -0.1, 0.1));
		c.insert("x1", Vector_(2, -0.1, -0.1));
		c.insert("x2", Vector_(2, 0.1, -0.2));
		return c;
	}

	/* ************************************************************************* */
	VectorConfig createZeroDelta() {
		VectorConfig c;
		c.insert("l1", zero(2));
		c.insert("x1", zero(2));
		c.insert("x2", zero(2));
		return c;
	}

	/* ************************************************************************* */
	GaussianFactorGraph createGaussianFactorGraph() {
		Matrix I = eye(2);

		// Create empty graph
		GaussianFactorGraph fg;

		// linearized prior on x1: c["x1"]+x1=0 i.e. x1=-c["x1"]
		Vector b1 = -Vector_(2,0.1, 0.1);
		//fg.add("x1", I, b1, sigma0_1);
		fg.add("x1", 10*eye(2), -1.0*ones(2), noiseModel::Unit::Create(2));

		// odometry between x1 and x2: x2-x1=[0.2;-0.1]
		Vector b2 = Vector_(2, 0.2, -0.1);
		//fg.add("x1", -I, "x2", I, b2, sigma0_1);
		fg.add("x1", -10*eye(2),"x2", 10*eye(2), Vector_(2, 2.0, -1.0),
				noiseModel::Unit::Create(2));

		// measurement between x1 and l1: l1-x1=[0.0;0.2]
		Vector b3 = Vector_(2, 0.0, 0.2);
		//fg.add("x1", -I, "l1", I, b3, sigma0_2);
		fg.add("x1", -5*eye(2),
				"l1", 5*eye(2), Vector_(2, 0.0, 1.0),
				noiseModel::Unit::Create(2));

		// measurement between x2 and l1: l1-x2=[-0.2;0.3]
		Vector b4 = Vector_(2, -0.2, 0.3);
		//fg.add("x2", -I, "l1", I, b4, sigma0_2);
		fg.add("x2", -5*eye(2),
				"l1", 5*eye(2), Vector_(2, -1.0, 1.5),
				noiseModel::Unit::Create(2));

		return fg;
	}

	/* ************************************************************************* */
	/** create small Chordal Bayes Net x <- y
	 * x y d
	 * 1 1 9
	 *   1 5
	 */
	GaussianBayesNet createSmallGaussianBayesNet() {
		Matrix R11 = Matrix_(1, 1, 1.0), S12 = Matrix_(1, 1, 1.0);
		Matrix R22 = Matrix_(1, 1, 1.0);
		Vector d1(1), d2(1);
		d1(0) = 9;
		d2(0) = 5;
		Vector tau(1);
		tau(0) = 1.0;

		// define nodes and specify in reverse topological sort (i.e. parents last)
		GaussianConditional::shared_ptr Px_y(new GaussianConditional("x", d1, R11,
				"y", S12, tau)), Py(new GaussianConditional("y", d2, R22, tau));
		GaussianBayesNet cbn;
		cbn.push_back(Px_y);
		cbn.push_back(Py);

		return cbn;
	}

	/* ************************************************************************* */
	// Some nonlinear functions to optimize
	/* ************************************************************************* */
	namespace smallOptimize {

		Point2 h(const Point2& v) {
			return Point2(cos(v.x()), sin(v.y()));
		}

		Matrix H(const Point2& v) {
			return Matrix_(2, 2,
					-sin(v.x()), 0.0,
					 0.0, cos(v.y()));
		}

		struct UnaryFactor: public gtsam::NonlinearFactor1<Config,
		simulated2D::PoseKey, Point2> {

			Point2 z_;

			UnaryFactor(const Point2& z, const SharedGaussian& model,
					const simulated2D::PoseKey& key) :
				gtsam::NonlinearFactor1<Config, simulated2D::PoseKey,
						Point2>(model, key), z_(z) {
			}

			Vector evaluateError(const Point2& x, boost::optional<Matrix&> A =
					boost::none) const {
				if (A) *A = H(x);
				return (h(x) - z_).vector();
			}

		};

	}

	/* ************************************************************************* */
	boost::shared_ptr<const Graph> sharedReallyNonlinearFactorGraph() {
		boost::shared_ptr<Graph> fg(
				new Graph);
		Vector z = Vector_(2, 1.0, 0.0);
		double sigma = 0.1;
		boost::shared_ptr<smallOptimize::UnaryFactor> factor(
				new smallOptimize::UnaryFactor(z, noiseModel::Isotropic::Sigma(2,sigma), 1));
		fg->push_back(factor);
		return fg;
	}

	Graph createReallyNonlinearFactorGraph() {
		return *sharedReallyNonlinearFactorGraph();
	}

	/* ************************************************************************* */
	pair<Graph, Config> createNonlinearSmoother(int T) {

		// Create
		Graph nlfg;
		Config poses;

		// prior on x1
		Point2 x1(1.0, 0.0);
		shared prior(new simulated2D::Prior(x1, sigma1_0, 1));
		nlfg.push_back(prior);
		poses.insert(simulated2D::PoseKey(1), x1);

		for (int t = 2; t <= T; t++) {
			// odometry between x_t and x_{t-1}
			Point2 odo(1.0, 0.0);
			shared odometry(new simulated2D::Odometry(odo, sigma1_0, t - 1, t));
			nlfg.push_back(odometry);

			// measurement on x_t is like perfect GPS
			Point2 xt(t, 0);
			shared measurement(new simulated2D::Prior(xt, sigma1_0, t));
			nlfg.push_back(measurement);

			// initial estimate
			poses.insert(simulated2D::PoseKey(t), xt);
		}

		return make_pair(nlfg, poses);
	}

	/* ************************************************************************* */
	GaussianFactorGraph createSmoother(int T) {
		Graph nlfg;
		Config poses;
		boost::tie(nlfg, poses) = createNonlinearSmoother(T);

		return *nlfg.linearize(poses);
	}

	/* ************************************************************************* */
	GaussianFactorGraph createSimpleConstraintGraph() {
		// create unary factor
		// prior on "x", mean = [1,-1], sigma=0.1
		Matrix Ax = eye(2);
		Vector b1(2);
		b1(0) = 1.0;
		b1(1) = -1.0;
		GaussianFactor::shared_ptr f1(new GaussianFactor("x", Ax, b1, sigma0_1));

		// create binary constraint factor
		// between "x" and "y", that is going to be the only factor on "y"
		// |1 0||x_1| + |-1  0||y_1| = |0|
		// |0 1||x_2|   | 0 -1||y_2|   |0|
		Matrix Ax1 = eye(2);
		Matrix Ay1 = eye(2) * -1;
		Vector b2 = Vector_(2, 0.0, 0.0);
		GaussianFactor::shared_ptr f2(new GaussianFactor("x", Ax1, "y", Ay1, b2,
				constraintModel));

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
		Matrix Ax = eye(2);
		Vector b1(2);
		b1(0) = 1.0;
		b1(1) = -1.0;
		//GaussianFactor::shared_ptr f1(new GaussianFactor("x", sigma0_1->Whiten(Ax), sigma0_1->whiten(b1), sigma0_1));
		GaussianFactor::shared_ptr f1(new GaussianFactor("x", Ax, b1, sigma0_1));

		// create binary constraint factor
		// between "x" and "y", that is going to be the only factor on "y"
		// |1 2||x_1| + |10 0||y_1| = |1|
		// |2 1||x_2|   |0 10||y_2|   |2|
		Matrix Ax1(2, 2);
		Ax1(0, 0) = 1.0;
		Ax1(0, 1) = 2.0;
		Ax1(1, 0) = 2.0;
		Ax1(1, 1) = 1.0;
		Matrix Ay1 = eye(2) * 10;
		Vector b2 = Vector_(2, 1.0, 2.0);
		GaussianFactor::shared_ptr f2(new GaussianFactor("x", Ax1, "y", Ay1, b2,
				constraintModel));

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
		config.insert("y", Vector_(2, 0.2, 0.1));
		return config;
	}

	/* ************************************************************************* */
	GaussianFactorGraph createMultiConstraintGraph() {
		// unary factor 1
		Matrix A = eye(2);
		Vector b = Vector_(2, -2.0, 2.0);
		GaussianFactor::shared_ptr lf1(new GaussianFactor("x", A, b, sigma0_1));

		// constraint 1
		Matrix A11(2, 2);
		A11(0, 0) = 1.0;
		A11(0, 1) = 2.0;
		A11(1, 0) = 2.0;
		A11(1, 1) = 1.0;

		Matrix A12(2, 2);
		A12(0, 0) = 10.0;
		A12(0, 1) = 0.0;
		A12(1, 0) = 0.0;
		A12(1, 1) = 10.0;

		Vector b1(2);
		b1(0) = 1.0;
		b1(1) = 2.0;
		GaussianFactor::shared_ptr lc1(new GaussianFactor("x", A11, "y", A12, b1,
				constraintModel));

		// constraint 2
		Matrix A21(2, 2);
		A21(0, 0) = 3.0;
		A21(0, 1) = 4.0;
		A21(1, 0) = -1.0;
		A21(1, 1) = -2.0;

		Matrix A22(2, 2);
		A22(0, 0) = 1.0;
		A22(0, 1) = 1.0;
		A22(1, 0) = 1.0;
		A22(1, 1) = 2.0;

		Vector b2(2);
		b2(0) = 3.0;
		b2(1) = 4.0;
		GaussianFactor::shared_ptr lc2(new GaussianFactor("x", A21, "z", A22, b2,
				constraintModel));

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

	/* ************************************************************************* */
	// Create key for simulated planar graph
	simulated2D::PoseKey key(int x, int y) {
		return simulated2D::PoseKey(1000*x+y);
	}

	/* ************************************************************************* */
	pair<GaussianFactorGraph, VectorConfig> planarGraph(size_t N) {

		// create empty graph
		NonlinearFactorGraph<Config> nlfg;

		// Create almost hard constraint on x11, sigma=0 will work for PCG not for normal
		shared constraint(new simulated2D::Prior(Point2(1.0, 1.0), sharedSigma(2,1e-3), key(1,1)));
		nlfg.push_back(constraint);

		// Create horizontal constraints, 1...N*(N-1)
		Point2 z1(1.0, 0.0); // move right
		for (size_t x = 1; x < N; x++)
			for (size_t y = 1; y <= N; y++) {
				shared f(new simulated2D::Odometry(z1, sharedSigma(2,0.01), key(x, y), key(x + 1, y)));
				nlfg.push_back(f);
			}

		// Create vertical constraints, N*(N-1)+1..2*N*(N-1)
		Point2 z2(0.0, 1.0); // move up
		for (size_t x = 1; x <= N; x++)
			for (size_t y = 1; y < N; y++) {
				shared f(new simulated2D::Odometry(z2, sharedSigma(2,0.01), key(x, y), key(x, y + 1)));
				nlfg.push_back(f);
			}

		// Create linearization and ground xtrue config
		Config zeros;
		VectorConfig xtrue;
		Point2 zero;
		for (size_t x = 1; x <= N; x++)
			for (size_t y = 1; y <= N; y++) {
				zeros.insert(key(x, y), zero);
				xtrue.insert((Symbol)key(x, y), Point2(x,y).vector());
			}

		// linearize around zero
		return make_pair(*nlfg.linearize(zeros), xtrue);
	}

	/* ************************************************************************* */
	Ordering planarOrdering(size_t N) {
		Ordering ordering;
		for (size_t y = N; y >= 1; y--)
			for (size_t x = N; x >= 1; x--)
				ordering.push_back(key(x, y));
		return ordering;
	}

	/* ************************************************************************* */
	pair<GaussianFactorGraph, GaussianFactorGraph> splitOffPlanarTree(size_t N,
			const GaussianFactorGraph& original) {
		GaussianFactorGraph T, C;

		// Add the x11 constraint to the tree
		T.push_back(original[0]);

		// Add all horizontal constraints to the tree
		size_t i = 1;
		for (size_t x = 1; x < N; x++)
			for (size_t y = 1; y <= N; y++, i++)
				T.push_back(original[i]);

		// Add first vertical column of constraints to T, others to C
		for (size_t x = 1; x <= N; x++)
			for (size_t y = 1; y < N; y++, i++)
				if (x == 1)
					T.push_back(original[i]);
				else
					C.push_back(original[i]);

		return make_pair(T, C);
	}

/* ************************************************************************* */

} // example
} // namespace gtsam

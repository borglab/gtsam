/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
#include <boost/shared_ptr.hpp>

using namespace std;

#define GTSAM_MAGIC_KEY

#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/smallExample.h>

// template definitions
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/TupleValues-inl.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>

namespace gtsam {
namespace example {

	typedef boost::shared_ptr<NonlinearFactor<Values> > shared;

	static SharedDiagonal sigma1_0 = noiseModel::Isotropic::Sigma(2,1.0);
	static SharedDiagonal sigma0_1 = noiseModel::Isotropic::Sigma(2,0.1);
	static SharedDiagonal sigma0_2 = noiseModel::Isotropic::Sigma(2,0.2);
	static SharedDiagonal constraintModel = noiseModel::Constrained::All(2);

	static const Index _l1_=0, _x1_=1, _x2_=2;
	static const Index _x_=0, _y_=1, _z_=2;

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
	Values createValues() {
		Values c;
		c.insert(simulated2D::PoseKey(1), Point2(0.0, 0.0));
		c.insert(simulated2D::PoseKey(2), Point2(1.5, 0.0));
		c.insert(simulated2D::PointKey(1), Point2(0.0, -1.0));
		return c;
	}

	/* ************************************************************************* */
	VectorValues createVectorValues() {
		VectorValues c(vector<size_t>(3, 2));
		c[_l1_] = Vector_(2, 0.0, -1.0);
		c[_x1_] = Vector_(2, 0.0, 0.0);
		c[_x2_] = Vector_(2, 1.5, 0.0);
		return c;
	}

	/* ************************************************************************* */
	boost::shared_ptr<const Values> sharedNoisyValues() {
		boost::shared_ptr<Values> c(new Values);
		c->insert(simulated2D::PoseKey(1), Point2(0.1, 0.1));
		c->insert(simulated2D::PoseKey(2), Point2(1.4, 0.2));
		c->insert(simulated2D::PointKey(1), Point2(0.1, -1.1));
		return c;
	}

	/* ************************************************************************* */
	Values createNoisyValues() {
		return *sharedNoisyValues();
	}

	/* ************************************************************************* */
	VectorValues createCorrectDelta(const Ordering& ordering) {
		VectorValues c(vector<size_t>(3,2));
		c[ordering["l1"]] = Vector_(2, -0.1, 0.1);
		c[ordering["x1"]] = Vector_(2, -0.1, -0.1);
		c[ordering["x2"]] = Vector_(2, 0.1, -0.2);
		return c;
	}

	/* ************************************************************************* */
	VectorValues createZeroDelta(const Ordering& ordering) {
		VectorValues c(vector<size_t>(3,2));
		c[ordering["l1"]] = zero(2);
		c[ordering["x1"]] = zero(2);
		c[ordering["x2"]] = zero(2);
		return c;
	}

	/* ************************************************************************* */
	FactorGraph<JacobianFactor> createGaussianFactorGraph(const Ordering& ordering) {
		Matrix I = eye(2);

		// Create empty graph
		GaussianFactorGraph fg;

		SharedDiagonal unit2 = noiseModel::Unit::Create(2);

		// linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
		fg.add(ordering["x1"], 10*eye(2), -1.0*ones(2), unit2);

		// odometry between x1 and x2: x2-x1=[0.2;-0.1]
		if(ordering["x1"] < ordering["x2"])
		  fg.add(ordering["x1"], -10*eye(2),ordering["x2"], 10*eye(2), Vector_(2, 2.0, -1.0), unit2);
		else
		  fg.add(ordering["x2"], 10*eye(2),ordering["x1"], -10*eye(2), Vector_(2, 2.0, -1.0), unit2);

    // measurement between x1 and l1: l1-x1=[0.0;0.2]
		if(ordering["x1"] < ordering["l1"])
		  fg.add(ordering["x1"], -5*eye(2), ordering["l1"], 5*eye(2), Vector_(2, 0.0, 1.0), unit2);
		else
		  fg.add(ordering["l1"], 5*eye(2), ordering["x1"], -5*eye(2), Vector_(2, 0.0, 1.0), unit2);

		// measurement between x2 and l1: l1-x2=[-0.2;0.3]
		if(ordering["x2"] < ordering["l1"])
		  fg.add(ordering["x2"], -5*eye(2), ordering["l1"], 5*eye(2), Vector_(2, -1.0, 1.5), unit2);
		else
		  fg.add(ordering["l1"], 5*eye(2), ordering["x2"], -5*eye(2), Vector_(2, -1.0, 1.5), unit2);

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
		GaussianConditional::shared_ptr Px_y(new GaussianConditional(_x_, d1, R11, _y_, S12, tau));
		GaussianConditional::shared_ptr Py(new GaussianConditional(_y_, d2, R22, tau));
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

		struct UnaryFactor: public gtsam::NonlinearFactor1<Values,
		simulated2D::PoseKey> {

			Point2 z_;

			UnaryFactor(const Point2& z, const SharedGaussian& model,
					const simulated2D::PoseKey& key) :
				gtsam::NonlinearFactor1<Values, simulated2D::PoseKey>(model, key), z_(z) {
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
		boost::shared_ptr<Graph> fg(new Graph);
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
	pair<Graph, Values> createNonlinearSmoother(int T) {

		// Create
		Graph nlfg;
		Values poses;

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
	pair<FactorGraph<JacobianFactor>, Ordering> createSmoother(int T, boost::optional<Ordering> ordering) {
		Graph nlfg;
		Values poses;
		boost::tie(nlfg, poses) = createNonlinearSmoother(T);

		if(!ordering) ordering = *poses.orderingArbitrary();
		return make_pair(*nlfg.linearize(poses, *ordering), *ordering);
	}

	/* ************************************************************************* */
	FactorGraph<JacobianFactor> createSimpleConstraintGraph() {
		// create unary factor
		// prior on _x_, mean = [1,-1], sigma=0.1
		Matrix Ax = eye(2);
		Vector b1(2);
		b1(0) = 1.0;
		b1(1) = -1.0;
		GaussianFactor::shared_ptr f1(new JacobianFactor(_x_, Ax, b1, sigma0_1));

		// create binary constraint factor
		// between _x_ and _y_, that is going to be the only factor on _y_
		// |1 0||x_1| + |-1  0||y_1| = |0|
		// |0 1||x_2|   | 0 -1||y_2|   |0|
		Matrix Ax1 = eye(2);
		Matrix Ay1 = eye(2) * -1;
		Vector b2 = Vector_(2, 0.0, 0.0);
		GaussianFactor::shared_ptr f2(new JacobianFactor(_x_, Ax1, _y_, Ay1, b2,
				constraintModel));

		// construct the graph
		FactorGraph<JacobianFactor> fg;
		fg.push_back(f1);
		fg.push_back(f2);

		return fg;
	}

	/* ************************************************************************* */
	VectorValues createSimpleConstraintValues() {
		VectorValues config(vector<size_t>(2,2));
		Vector v = Vector_(2, 1.0, -1.0);
		config[_x_] = v;
		config[_y_] = v;
		return config;
	}

	/* ************************************************************************* */
	FactorGraph<JacobianFactor> createSingleConstraintGraph() {
		// create unary factor
		// prior on _x_, mean = [1,-1], sigma=0.1
		Matrix Ax = eye(2);
		Vector b1(2);
		b1(0) = 1.0;
		b1(1) = -1.0;
		//GaussianFactor::shared_ptr f1(new JacobianFactor(_x_, sigma0_1->Whiten(Ax), sigma0_1->whiten(b1), sigma0_1));
		GaussianFactor::shared_ptr f1(new JacobianFactor(_x_, Ax, b1, sigma0_1));

		// create binary constraint factor
		// between _x_ and _y_, that is going to be the only factor on _y_
		// |1 2||x_1| + |10 0||y_1| = |1|
		// |2 1||x_2|   |0 10||y_2|   |2|
		Matrix Ax1(2, 2);
		Ax1(0, 0) = 1.0;
		Ax1(0, 1) = 2.0;
		Ax1(1, 0) = 2.0;
		Ax1(1, 1) = 1.0;
		Matrix Ay1 = eye(2) * 10;
		Vector b2 = Vector_(2, 1.0, 2.0);
		GaussianFactor::shared_ptr f2(new JacobianFactor(_x_, Ax1, _y_, Ay1, b2,
				constraintModel));

		// construct the graph
		FactorGraph<JacobianFactor> fg;
		fg.push_back(f1);
		fg.push_back(f2);

		return fg;
	}

	/* ************************************************************************* */
	VectorValues createSingleConstraintValues() {
		VectorValues config(vector<size_t>(2,2));
		config[_x_] = Vector_(2, 1.0, -1.0);
		config[_y_] = Vector_(2, 0.2, 0.1);
		return config;
	}

	/* ************************************************************************* */
	FactorGraph<JacobianFactor> createMultiConstraintGraph() {
		// unary factor 1
		Matrix A = eye(2);
		Vector b = Vector_(2, -2.0, 2.0);
		GaussianFactor::shared_ptr lf1(new JacobianFactor(_x_, A, b, sigma0_1));

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
		GaussianFactor::shared_ptr lc1(new JacobianFactor(_x_, A11, _y_, A12, b1,
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
		GaussianFactor::shared_ptr lc2(new JacobianFactor(_x_, A21, _z_, A22, b2,
				constraintModel));

		// construct the graph
		FactorGraph<JacobianFactor> fg;
		fg.push_back(lf1);
		fg.push_back(lc1);
		fg.push_back(lc2);

		return fg;
	}

	/* ************************************************************************* */
	VectorValues createMultiConstraintValues() {
		VectorValues config(vector<size_t>(3,2));
		config[_x_] = Vector_(2, -2.0, 2.0);
		config[_y_] = Vector_(2, -0.1, 0.4);
		config[_z_] = Vector_(2, -4.0, 5.0);
		return config;
	}


	/* ************************************************************************* */
	// Create key for simulated planar graph
	simulated2D::PoseKey key(int x, int y) {
		return simulated2D::PoseKey(1000*x+y);
	}

	/* ************************************************************************* */
	boost::tuple<FactorGraph<JacobianFactor>, Ordering, VectorValues> planarGraph(size_t N) {

		// create empty graph
		NonlinearFactorGraph<Values> nlfg;

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
		Values zeros;
    for (size_t x = 1; x <= N; x++)
      for (size_t y = 1; y <= N; y++)
        zeros.insert(key(x, y), Point2());
		Ordering ordering(planarOrdering(N));
		VectorValues xtrue(zeros.dims(ordering));
		for (size_t x = 1; x <= N; x++)
			for (size_t y = 1; y <= N; y++)
				xtrue[ordering[key(x, y)]] = Point2(x,y).vector();

		// linearize around zero
		return boost::make_tuple(*nlfg.linearize(zeros, ordering), ordering, xtrue);
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
	pair<FactorGraph<JacobianFactor>, FactorGraph<JacobianFactor> > splitOffPlanarTree(size_t N,
			const FactorGraph<JacobianFactor>& original) {
		FactorGraph<JacobianFactor> T, C;

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

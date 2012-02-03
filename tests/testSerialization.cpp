/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief Unit tests for serialization of library classes
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

/* ************************************************************************* */
// Serialization testing code.
/* ************************************************************************* */

#include <sstream>
#include <string>

// includes for standard serialization types
#include <boost/serialization/export.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

// whether to print the serialized text to stdout
const bool verbose = false;

template<class T>
std::string serialize(const T& input) {
	std::ostringstream out_archive_stream;
	boost::archive::text_oarchive out_archive(out_archive_stream);
	out_archive << input;
	return out_archive_stream.str();
}

template<class T>
void deserialize(const std::string& serialized, T& output) {
	std::istringstream in_archive_stream(serialized);
	boost::archive::text_iarchive in_archive(in_archive_stream);
	in_archive >> output;
}

// Templated round-trip serialization
template<class T>
void roundtrip(const T& input, T& output) {
	// Serialize
	std::string serialized = serialize(input);
	if (verbose) std::cout << serialized << std::endl << std::endl;

	deserialize(serialized, output);
}

// This version requires equality operator
template<class T>
bool equality(const T& input = T()) {
	T output;
	roundtrip<T>(input,output);
	return input==output;
}

// This version requires equals
template<class T>
bool equalsObj(const T& input = T()) {
	T output;
	roundtrip<T>(input,output);
	return input.equals(output);
}

// De-referenced version for pointers
template<class T>
bool equalsDereferenced(const T& input) {
	T output;
	roundtrip<T>(input,output);
	return input->equals(*output);
}

/* ************************************************************************* */
template<class T>
std::string serializeXML(const T& input) {
	std::ostringstream out_archive_stream;
	boost::archive::xml_oarchive out_archive(out_archive_stream);
	out_archive << boost::serialization::make_nvp("data", input);
	return out_archive_stream.str();
}

template<class T>
void deserializeXML(const std::string& serialized, T& output) {
	std::istringstream in_archive_stream(serialized);
	boost::archive::xml_iarchive in_archive(in_archive_stream);
	in_archive >> boost::serialization::make_nvp("data", output);
}

// Templated round-trip serialization using XML
template<class T>
void roundtripXML(const T& input, T& output) {
	// Serialize
	std::string serialized = serializeXML<T>(input);
	if (verbose) std::cout << serialized << std::endl << std::endl;

	// De-serialize
	deserializeXML(serialized, output);
}

// This version requires equality operator
template<class T>
bool equalityXML(const T& input = T()) {
	T output;
	roundtripXML<T>(input,output);
	return input==output;
}

// This version requires equals
template<class T>
bool equalsXML(const T& input = T()) {
	T output;
	roundtripXML<T>(input,output);
	return input.equals(output);
}

// This version is for pointers
template<class T>
bool equalsDereferencedXML(const T& input = T()) {
	T output;
	roundtripXML<T>(input,output);
	return input->equals(*output);
}

/* ************************************************************************* */
// Actual Tests
/* ************************************************************************* */

// Magically casts strings like "x3" to a Symbol('x',3) key, see Key.h
#define GTSAM_MAGIC_KEY

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/pose3SLAM.h>
#include <gtsam/slam/visualSLAM.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST (Serialization, matrix_vector) {
	EXPECT(equality<Vector>(Vector_(4, 1.0, 2.0, 3.0, 4.0)));
	EXPECT(equality<Matrix>(Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0)));

	EXPECT(equalityXML<Vector>(Vector_(4, 1.0, 2.0, 3.0, 4.0)));
	EXPECT(equalityXML<Matrix>(Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0)));
}

Point3 pt3(1.0, 2.0, 3.0);
Rot3 rt3 = Rot3::RzRyRx(1.0, 3.0, 2.0);
Pose3 pose3(rt3, pt3);

Cal3_S2 cal1(1.0, 2.0, 0.3, 0.1, 0.5);
Cal3DS2 cal2(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
Cal3Bundler cal3(1.0, 2.0, 3.0);
Cal3_S2Stereo cal4(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
Cal3_S2Stereo::shared_ptr cal4ptr(new Cal3_S2Stereo(cal4));
CalibratedCamera cal5(Pose3(rt3, pt3));

PinholeCamera<Cal3_S2> cam1(pose3, cal1);
StereoCamera cam2(pose3, cal4ptr);
StereoPoint2 spt(1.0, 2.0, 3.0);


/* ************************************************************************* */
TEST (Serialization, text_geometry) {
	EXPECT(equalsObj<gtsam::Point2>(Point2(1.0, 2.0)));
	EXPECT(equalsObj<gtsam::Pose2>(Pose2(1.0, 2.0, 0.3)));
	EXPECT(equalsObj<gtsam::Rot2>(Rot2::fromDegrees(30.0)));

	EXPECT(equalsObj(pt3));
	EXPECT(equalsObj<gtsam::Rot3>(rt3));
	EXPECT(equalsObj<gtsam::Pose3>(Pose3(rt3, pt3)));

	EXPECT(equalsObj(cal1));
	EXPECT(equalsObj(cal2));
	EXPECT(equalsObj(cal3));
	EXPECT(equalsObj(cal4));
	EXPECT(equalsObj(cal5));

	EXPECT(equalsObj(cam1));
	EXPECT(equalsObj(cam2));
	EXPECT(equalsObj(spt));
}

/* ************************************************************************* */
TEST (Serialization, xml_geometry) {
	EXPECT(equalsXML<gtsam::Point2>(Point2(1.0, 2.0)));
	EXPECT(equalsXML<gtsam::Pose2>(Pose2(1.0, 2.0, 0.3)));
	EXPECT(equalsXML<gtsam::Rot2>(Rot2::fromDegrees(30.0)));

	EXPECT(equalsXML<gtsam::Point3>(pt3));
	EXPECT(equalsXML<gtsam::Rot3>(rt3));
	EXPECT(equalsXML<gtsam::Pose3>(Pose3(rt3, pt3)));

	EXPECT(equalsXML(cal1));
	EXPECT(equalsXML(cal2));
	EXPECT(equalsXML(cal3));
	EXPECT(equalsXML(cal4));
	EXPECT(equalsXML(cal5));

	EXPECT(equalsXML(cam1));
	EXPECT(equalsXML(cam2));
	EXPECT(equalsXML(spt));
}

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
noiseModel::Diagonal::shared_ptr diag3 = noiseModel::Diagonal::Sigmas(Vector_(3, 0.1, 0.2, 0.3));
noiseModel::Gaussian::shared_ptr gaussian3 = noiseModel::Gaussian::SqrtInformation(2.0 * eye(3,3));
noiseModel::Isotropic::shared_ptr iso3 = noiseModel::Isotropic::Sigma(3, 0.2);
noiseModel::Constrained::shared_ptr constrained3 = noiseModel::Constrained::MixedSigmas(Vector_(3, 0.0, 0.0, 0.1));
noiseModel::Unit::shared_ptr unit3 = noiseModel::Unit::Create(3);

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

/* Create GUIDs for factors */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor , "gtsam::HessianFactor");
/* ************************************************************************* */
TEST (Serialization, smallExample_linear) {
	using namespace example;

  Ordering ordering; ordering += "x1","x2","l1";
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  EXPECT(equalsObj(ordering));
  EXPECT(equalsXML(ordering));

  EXPECT(equalsObj(fg));
  EXPECT(equalsXML(fg));

  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  EXPECT(equalsObj(cbn));
  EXPECT(equalsXML(cbn));
}

/* ************************************************************************* */
TEST (Serialization, symbolic_graph) {
  Ordering o; o += "x1","l1","x2";
	// construct expected symbolic graph
	SymbolicFactorGraph sfg;
	sfg.push_factor(o["x1"]);
	sfg.push_factor(o["x1"],o["x2"]);
	sfg.push_factor(o["x1"],o["l1"]);
	sfg.push_factor(o["l1"],o["x2"]);

	EXPECT(equalsObj(sfg));
	EXPECT(equalsXML(sfg));
}

/* ************************************************************************* */
TEST (Serialization, symbolic_bn) {
  Ordering o; o += "x2","l1","x1";

  IndexConditional::shared_ptr x2(new IndexConditional(o["x2"], o["l1"], o["x1"]));
  IndexConditional::shared_ptr l1(new IndexConditional(o["l1"], o["x1"]));
  IndexConditional::shared_ptr x1(new IndexConditional(o["x1"]));

  SymbolicBayesNet sbn;
  sbn.push_back(x2);
  sbn.push_back(l1);
  sbn.push_back(x1);

	EXPECT(equalsObj(sbn));
	EXPECT(equalsXML(sbn));
}

/* ************************************************************************* */
TEST (Serialization, symbolic_bayes_tree ) {
	typedef BayesTree<IndexConditional> SymbolicBayesTree;
	static const Index _X_=0, _T_=1, _S_=2, _E_=3, _L_=4, _B_=5;
	IndexConditional::shared_ptr
	B(new IndexConditional(_B_)),
	L(new IndexConditional(_L_, _B_)),
	E(new IndexConditional(_E_, _L_, _B_)),
	S(new IndexConditional(_S_, _L_, _B_)),
	T(new IndexConditional(_T_, _E_, _L_)),
	X(new IndexConditional(_X_, _E_));

	// Bayes Tree for Asia example
	SymbolicBayesTree bayesTree;
	SymbolicBayesTree::insert(bayesTree, B);
	SymbolicBayesTree::insert(bayesTree, L);
	SymbolicBayesTree::insert(bayesTree, E);
	SymbolicBayesTree::insert(bayesTree, S);
	SymbolicBayesTree::insert(bayesTree, T);
	SymbolicBayesTree::insert(bayesTree, X);

	EXPECT(equalsObj(bayesTree));
	EXPECT(equalsXML(bayesTree));
}

/* ************************************************************************* */
TEST (Serialization, gaussianISAM) {
	using namespace example;
  Ordering ordering;
	GaussianFactorGraph smoother;
	boost::tie(smoother, ordering) = createSmoother(7);
	BayesTree<GaussianConditional> bayesTree = *GaussianMultifrontalSolver(smoother).eliminate();
	GaussianISAM isam(bayesTree);

	EXPECT(equalsObj(isam));
	EXPECT(equalsXML(isam));
}

/* ************************************************************************* */
/* Create GUIDs for factors in simulated2D */
BOOST_CLASS_EXPORT_GUID(simulated2D::Prior,       "gtsam::simulated2D::Prior"      );
BOOST_CLASS_EXPORT_GUID(simulated2D::Odometry,    "gtsam::simulated2D::Odometry"   );
BOOST_CLASS_EXPORT_GUID(simulated2D::Measurement, "gtsam::simulated2D::Measurement");
/* ************************************************************************* */
TEST (Serialization, smallExample) {
	using namespace example;
	Graph nfg = createNonlinearFactorGraph();
	Values c1 = createValues();
	EXPECT(equalsObj(nfg));
	EXPECT(equalsXML(nfg));

	EXPECT(equalsObj(c1));
	EXPECT(equalsXML(c1));
}

/* ************************************************************************* */
/* Create GUIDs for factors */
BOOST_CLASS_EXPORT_GUID(planarSLAM::Prior,       "gtsam::planarSLAM::Prior");
BOOST_CLASS_EXPORT_GUID(planarSLAM::Bearing,     "gtsam::planarSLAM::Bearing");
BOOST_CLASS_EXPORT_GUID(planarSLAM::Range,       "gtsam::planarSLAM::Range");
BOOST_CLASS_EXPORT_GUID(planarSLAM::BearingRange,"gtsam::planarSLAM::BearingRange");
BOOST_CLASS_EXPORT_GUID(planarSLAM::Odometry,    "gtsam::planarSLAM::Odometry");
BOOST_CLASS_EXPORT_GUID(planarSLAM::Constraint,  "gtsam::planarSLAM::Constraint");
/* ************************************************************************* */
TEST (Serialization, planar_system) {
	using namespace planarSLAM;
	Values values;
	values.insert(PointKey(3), Point2(1.0, 2.0));
	values.insert(PoseKey(4), Pose2(1.0, 2.0, 0.3));

	SharedNoiseModel model1 = noiseModel::Isotropic::Sigma(1, 0.3);
	SharedNoiseModel model2 = noiseModel::Isotropic::Sigma(2, 0.3);
	SharedNoiseModel model3 = noiseModel::Isotropic::Sigma(3, 0.3);

	Prior prior(PoseKey(3), Pose2(0.1,-0.3, 0.2), model1);
	Bearing bearing(PoseKey(3), PointKey(5), Rot2::fromDegrees(0.5), model1);
	Range range(PoseKey(2), PointKey(9), 7.0, model1);
	BearingRange bearingRange(PoseKey(2), PointKey(3), Rot2::fromDegrees(0.6), 2.0, model2);
	Odometry odometry(PoseKey(2), PoseKey(3), Pose2(1.0, 2.0, 0.3), model3);
	Constraint constraint(PoseKey(9), Pose2(2.0,-1.0, 0.2));

	Graph graph;
	graph.add(prior);
	graph.add(bearing);
	graph.add(range);
	graph.add(bearingRange);
	graph.add(odometry);
	graph.add(constraint);

	// text
	EXPECT(equalsObj<PoseKey>(PoseKey(2)));
	EXPECT(equalsObj<PointKey>(PointKey(3)));
	EXPECT(equalsObj<Values>(values));
	EXPECT(equalsObj<Prior>(prior));
	EXPECT(equalsObj<Bearing>(bearing));
	EXPECT(equalsObj<BearingRange>(bearingRange));
	EXPECT(equalsObj<Range>(range));
	EXPECT(equalsObj<Odometry>(odometry));
	EXPECT(equalsObj<Constraint>(constraint));
	EXPECT(equalsObj<Graph>(graph));

	// xml
	EXPECT(equalsXML<PoseKey>(PoseKey(2)));
	EXPECT(equalsXML<PointKey>(PointKey(3)));
	EXPECT(equalsXML<Values>(values));
	EXPECT(equalsXML<Prior>(prior));
	EXPECT(equalsXML<Bearing>(bearing));
	EXPECT(equalsXML<BearingRange>(bearingRange));
	EXPECT(equalsXML<Range>(range));
	EXPECT(equalsXML<Odometry>(odometry));
	EXPECT(equalsXML<Constraint>(constraint));
	EXPECT(equalsXML<Graph>(graph));
}

/* ************************************************************************* */
/* Create GUIDs for factors */
BOOST_CLASS_EXPORT_GUID(visualSLAM::PoseConstraint,  "gtsam::visualSLAM::PoseConstraint");
BOOST_CLASS_EXPORT_GUID(visualSLAM::PointConstraint, "gtsam::visualSLAM::PointConstraint");
BOOST_CLASS_EXPORT_GUID(visualSLAM::PosePrior,       "gtsam::visualSLAM::PosePrior");
BOOST_CLASS_EXPORT_GUID(visualSLAM::PointPrior,      "gtsam::visualSLAM::PointPrior");
BOOST_CLASS_EXPORT_GUID(visualSLAM::ProjectionFactor,"gtsam::visualSLAM::ProjectionFactor");
BOOST_CLASS_EXPORT_GUID(visualSLAM::StereoFactor,    "gtsam::visualSLAM::StereoFactor");

/* ************************************************************************* */
TEST (Serialization, visual_system) {
	using namespace visualSLAM;
	Values values;
	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);
	Pose3 pose1 = pose3, pose2 = pose3.inverse();
	Point3 pt1(1.0, 2.0, 3.0), pt2(4.0, 5.0, 6.0);
	values.insert(x1, pose1);
	values.insert(l1, pt1);
	SharedNoiseModel model2 = noiseModel::Isotropic::Sigma(2, 0.3);
	SharedNoiseModel model3 = noiseModel::Isotropic::Sigma(3, 0.3);
	SharedNoiseModel model6 = noiseModel::Isotropic::Sigma(6, 0.3);
	boost::shared_ptr<Cal3_S2> K(new Cal3_S2(cal1));

	Graph graph;
	graph.addMeasurement(Point2(1.0, 2.0), model2, x1, l1, K);
	graph.addPointConstraint(1, pt1);
	graph.addPointPrior(1, pt2, model3);
	graph.addPoseConstraint(1, pose1);
	graph.addPosePrior(1, pose2, model6);

	EXPECT(equalsObj(values));
	EXPECT(equalsObj(graph));

	EXPECT(equalsXML(values));
	EXPECT(equalsXML(graph));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

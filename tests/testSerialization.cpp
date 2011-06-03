/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @brief Unit tests for serialization of library classes
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

/* ************************************************************************* */
// Serialization testing code.
/* ************************************************************************* */

#include <fstream>
#include <sstream>
#include <string>

// includes for standard serialization types
#include <boost/serialization/export.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

// whether to print the serialized text to stdout
const bool verbose = false;

// Templated round-trip serialization
template<class T>
void roundtrip(const T& input, T& output) {
	// Serialize
	std::ostringstream out_archive_stream;
	{
		boost::archive::text_oarchive out_archive(out_archive_stream);
		out_archive << input;
	}

	// Convert to string
	std::string serialized = out_archive_stream.str();
	if (verbose) std::cout << serialized << std::endl << std::endl;

	// De-serialize
	{
		std::istringstream in_archive_stream(serialized);
		boost::archive::text_iarchive in_archive(in_archive_stream);
		in_archive >> output;
	}
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
// Templated round-trip serialization using XML
template<class T>
void roundtripXML(const T& input, T& output) {
	// Serialize
	std::string serialized;
	{
		std::ostringstream out_archive_stream;
		boost::archive::xml_oarchive out_archive(out_archive_stream);
		out_archive << boost::serialization::make_nvp("data", input);

		// Convert to string
		serialized = out_archive_stream.str();
	}
	if (verbose) std::cout << serialized << std::endl << std::endl;

	// De-serialize
	std::istringstream in_archive_stream(serialized);
	boost::archive::xml_iarchive in_archive(in_archive_stream);
	in_archive >> boost::serialization::make_nvp("data", output);
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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/StereoCamera.h>

#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/pose3SLAM.h>
#include <gtsam/slam/visualSLAM.h>
#include <gtsam/slam/BearingFactor.h>

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
CalibratedCamera cal5(Pose3(rt3, pt3));

SimpleCamera cam1(cal1, cal5);
StereoCamera cam2(pose3, cal4);
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

BOOST_CLASS_EXPORT_GUID(gtsam::SharedGaussian, "gtsam_SharedGaussian");
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
TEST (Serialization, SharedGaussian_noiseModels) {
	SharedGaussian diag3_sg = diag3;
	EXPECT(equalsDereferenced<SharedGaussian>(diag3_sg));
	EXPECT(equalsDereferencedXML<SharedGaussian>(diag3_sg));

	EXPECT(equalsDereferenced<SharedGaussian>(diag3));
	EXPECT(equalsDereferencedXML<SharedGaussian>(diag3));

	EXPECT(equalsDereferenced<SharedGaussian>(iso3));
	EXPECT(equalsDereferencedXML<SharedGaussian>(iso3));

	EXPECT(equalsDereferenced<SharedGaussian>(gaussian3));
	EXPECT(equalsDereferencedXML<SharedGaussian>(gaussian3));

	EXPECT(equalsDereferenced<SharedGaussian>(unit3));
	EXPECT(equalsDereferencedXML<SharedGaussian>(unit3));

	EXPECT(equalsDereferenced<SharedGaussian>(constrained3));
	EXPECT(equalsDereferencedXML<SharedGaussian>(constrained3));
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

#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>

/* ************************************************************************* */
TEST (Serialization, linear) {
	vector<size_t> dims;
	dims.push_back(1);
	dims.push_back(2);
	dims.push_back(2);
	double v[] = {1., 2., 3., 4., 5.};
	VectorValues values(dims, v);
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
	{
		Matrix A1 = Matrix_(2,2, 1., 2., 3., 4.);
		Matrix A2 = Matrix_(2,2, 6., 0.2, 8., 0.4);
		Matrix R = Matrix_(2,2, 0.1, 0.3, 0.0, 0.34);
		Vector d(2); d << 0.2, 0.5;
		GaussianConditional cg(0, d, R, 1, A1, 2, A2, ones(2));
//		EXPECT(equalsObj(cg)); // FAILS: does not match
//		EXPECT(equalsXML(cg)); // FAILS: does not match
	}
}

/* ************************************************************************* */
/* Create GUIDs for factors */
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Prior,       "gtsam::planarSLAM::Prior");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Bearing,     "gtsam::planarSLAM::Bearing");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Range,       "gtsam::planarSLAM::Range");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::BearingRange,"gtsam::planarSLAM::BearingRange");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Odometry,    "gtsam::planarSLAM::Odometry");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Constraint,  "gtsam::planarSLAM::Constraint");

/* ************************************************************************* */
TEST (Serialization, planar_system) {
	using namespace planarSLAM;
	Values values;
	values.insert(PointKey(3), Point2(1.0, 2.0));
	values.insert(PoseKey(4), Pose2(1.0, 2.0, 0.3));

	SharedGaussian model1 = noiseModel::Isotropic::Sigma(1, 0.3);
	SharedGaussian model2 = noiseModel::Isotropic::Sigma(2, 0.3);
	SharedGaussian model3 = noiseModel::Isotropic::Sigma(3, 0.3);

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
BOOST_CLASS_EXPORT_GUID(gtsam::visualSLAM::PoseConstraint,  "gtsam::visualSLAM::PoseConstraint");
BOOST_CLASS_EXPORT_GUID(gtsam::visualSLAM::PointConstraint, "gtsam::visualSLAM::PointConstraint");
BOOST_CLASS_EXPORT_GUID(gtsam::visualSLAM::PosePrior,       "gtsam::visualSLAM::PosePrior");
BOOST_CLASS_EXPORT_GUID(gtsam::visualSLAM::PointPrior,      "gtsam::visualSLAM::PointPrior");
BOOST_CLASS_EXPORT_GUID(gtsam::visualSLAM::ProjectionFactor,"gtsam::visualSLAM::ProjectionFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::visualSLAM::StereoFactor,    "gtsam::visualSLAM::StereoFactor");

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
	SharedGaussian model2 = noiseModel::Isotropic::Sigma(2, 0.3);
	SharedGaussian model3 = noiseModel::Isotropic::Sigma(3, 0.3);
	SharedGaussian model6 = noiseModel::Isotropic::Sigma(6, 0.3);
	boost::shared_ptr<Cal3_S2> K(new Cal3_S2(cal1));

	Graph graph;
	graph.addMeasurement(Point2(1.0, 2.0), model2, x1, l1, K);
	graph.addPointConstraint(1, pt1);
	graph.addPointPrior(1, pt2, model3);
	graph.addPoseConstraint(1, pose1);
	graph.addPosePrior(1, pose3, model6);

	EXPECT(equalsObj(values));
	EXPECT(equalsObj(graph));

	EXPECT(equalsXML(values));
	EXPECT(equalsXML(graph));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

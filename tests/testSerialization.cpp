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

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Cal3_S2.h>

//#include <gtsam/linear/VectorValues.h>
//#include <gtsam/linear/GaussianConditional.h>
//#include <gtsam/inference/SymbolicConditional.h>

#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/BearingFactor.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace planarSLAM;

/* ************************************************************************* */
TEST (Serialization, text_geometry) {
	EXPECT(equalsObj<gtsam::Point2>(Point2(1.0, 2.0)));
	EXPECT(equalsObj<gtsam::Pose2>(Pose2(1.0, 2.0, 0.3)));
	EXPECT(equalsObj<gtsam::Rot2>(Rot2::fromDegrees(30.0)));

	Point3 pt3(1.0, 2.0, 3.0);
	Rot3 rt3 = Rot3::RzRyRx(1.0, 3.0, 2.0);
	EXPECT(equalsObj<gtsam::Point3>(pt3));
	EXPECT(equalsObj<gtsam::Rot3>(rt3));
	EXPECT(equalsObj<gtsam::Pose3>(Pose3(rt3, pt3)));
}

/* ************************************************************************* */
TEST (Serialization, xml_geometry) {
	EXPECT(equalsXML<gtsam::Point2>(Point2(1.0, 2.0)));
	EXPECT(equalsXML<gtsam::Pose2>(Pose2(1.0, 2.0, 0.3)));
	EXPECT(equalsXML<gtsam::Rot2>(Rot2::fromDegrees(30.0)));

	Point3 pt3(1.0, 2.0, 3.0);
	Rot3 rt3 = Rot3::RzRyRx(1.0, 3.0, 2.0);
	EXPECT(equalsXML<gtsam::Point3>(pt3));
	EXPECT(equalsXML<gtsam::Rot3>(rt3));
	EXPECT(equalsXML<gtsam::Pose3>(Pose3(rt3, pt3)));
}

/* ************************************************************************* */
TEST (Serialization, text_linear) {
	// NOT WORKING:
//	EXPECT(equalsObj<VectorValues>());
//	EXPECT(equalsObj<GaussianConditional>());
}

/* ************************************************************************* */
TEST (Serialization, xml_linear) {
	// NOT WORKING:
//	EXPECT(equalsXML<VectorValues>());
//	EXPECT(equalsXML<GaussianConditional>());
}
/* ************************************************************************* */
// Export Noisemodels
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");

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
/* Create GUIDs for factors */
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Prior,       "gtsam::planarSLAM::Prior");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Bearing,     "gtsam::planarSLAM::Bearing");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Range,       "gtsam::planarSLAM::Range");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::BearingRange,"gtsam::planarSLAM::BearingRange");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Odometry,    "gtsam::planarSLAM::Odometry");
BOOST_CLASS_EXPORT_GUID(gtsam::planarSLAM::Constraint,  "gtsam::planarSLAM::Constraint");

/* ************************************************************************* */
TEST (Serialization, planar_system) {

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
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

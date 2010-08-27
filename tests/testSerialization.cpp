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
//#include <boost/serialization/vector.hpp>
//#include <boost/serialization/map.hpp>
//#include <boost/serialization/list.hpp>

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
	boost::archive::text_oarchive out_archive(out_archive_stream);
	out_archive << input;

	// Convert to string
	std::string serialized = out_archive_stream.str();
	if (verbose) std::cout << serialized << std::endl << std::endl;

	// De-serialize
	std::istringstream in_archive_stream(serialized);
	boost::archive::text_iarchive in_archive(in_archive_stream);
	in_archive >> output;
}

// This version requires equality operator
template<class T>
bool equality() {
	T input;
	T output;
	roundtrip<T>(input,output);
	return input==output;
}

template<class T>
bool equality(const T& input) {
	T output;
	roundtrip<T>(input,output);
	return input==output;
}

// This version requires equals
template<class T>
bool equalsEmpty() {
	T input;
	T output;
	roundtrip<T>(input,output);
	return input.equals(output);
}

template<class T>
bool equalsObj(const T& input) {
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
	std::ostringstream out_archive_stream;
	boost::archive::xml_oarchive out_archive(out_archive_stream);
	out_archive << BOOST_SERIALIZATION_NVP(input);

	// Convert to string
	std::string serialized = out_archive_stream.str();
	if (verbose) std::cout << serialized << std::endl << std::endl;

	// De-serialize
	std::istringstream in_archive_stream(serialized);
	boost::archive::xml_iarchive in_archive(in_archive_stream);
	in_archive >> BOOST_SERIALIZATION_NVP(output);
}

// This version requires equality operator
template<class T>
bool equalityXML() {
	T input;
	T output;
	roundtripXML<T>(input,output);
	return input==output;
}

template<class T>
bool equalityXML(const T& input) {
	T output;
	roundtripXML<T>(input,output);
	return input==output;
}

// This version requires equals
template<class T>
bool equalsXML() {
	T input;
	T output;
	roundtripXML<T>(input,output);
	return input.equals(output);
}

template<class T>
bool equalsXML(const T& input) {
	T output;
	roundtripXML<T>(input,output);
	return input.equals(output);
}

/* ************************************************************************* */
// Actual Tests
/* ************************************************************************* */

#include <gtsam/geometry/Point2.h>
//#include <gtsam/geometry/Pose2.h>
//#include <gtsam/geometry/Rot2.h>
//#include <gtsam/geometry/Point3.h>
//#include <gtsam/geometry/Pose3.h>
//#include <gtsam/geometry/Rot3.h>
//#include <gtsam/geometry/Cal3_S2.h>

//#include <gtsam/linear/VectorConfig.h>
//#include <gtsam/inference/FactorGraph-inl.h>
//#include <gtsam/linear/GaussianConditional.h>
//#include <gtsam/inference/SymbolicConditional.h>

#include <gtsam/CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
//TEST( Point2,              equalsEmpty) { CHECK(equalsEmpty<gtsam::Point2>());}
//TEST( VectorConfig,        equalsEmpty) { CHECK(equalsEmpty<VectorConfig>());}
//TEST( GaussianConditional, equalsEmpty) { CHECK(equalsEmpty<GaussianConditional>());}



/* ************************************************************************* */
// Testing XML
/* ************************************************************************* */
//TEST( Point2,          	   equalsXML) { CHECK(equalsXML<gtsam::Point2>());}

//TEST( VectorConfig,        equalsXML) { CHECK(equalsXML<VectorConfig>());}
//TEST( Cal3_S2,             equalsXML) { CHECK(equalsXML<Cal3_S2>());}
//TEST( GaussianConditional, equalsXML) { CHECK(equalsXML<GaussianConditional>());}
//TEST( SymbolicConditional, equalsXML) { CHECK(equalsXML<SymbolicConditional>());}


// The two tests below will not run, however, as CameraMarkerFactor
// is not registered by the serialization code in FactorGraph
//TEST( FactorGraph, equals1)
//{
//	FactorGraph<NonlinearFactor> graph;
//  graph.push_back(f);
//	CHECK(equalsObj(graph));
//}
//TEST( NonlinearFactorGraph, equals)
//{
//	NonlinearFactorGraph graph;
//  graph.push_back(f);
//	CHECK(equalsObj(graph));
//}

// It *does* work if we explicitly instantiate with the factor type
//TEST( FactorGraph, equals2)
//{
//	FactorGraph<CameraMarkerFactor> graph;
//  graph.push_back(f);
//	CHECK(equalsObj(graph));
//}

// And, as we explicitly registered the three CameraMarkerFactor types in
// the EasySLAMGraph serialize, this will also work :-))
// see http://www.boost.org/doc/libs/1_35_0/libs/serialization/doc/serialization.html#derivedpointers
//TEST( EasySLAMGraph, equals2)
//{
//	EasySLAMGraph graph = createExampleGraph();
//	CHECK(equalsObj(graph));
//}
//
//// EasySLAMConfig is no problem either:
//TEST( EasySLAMConfig, equals2)
//{
//	EasySLAMConfig c = createExampleConfig();
//	CHECK(equalsObj(c));
//}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

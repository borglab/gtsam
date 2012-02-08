/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file serializationTestHelpers.h
 * @brief 
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#pragma once

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

namespace gtsam { namespace serializationTestHelpers {

  /* ************************************************************************* */
  // Serialization testing code.
  /* ************************************************************************* */

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

} }

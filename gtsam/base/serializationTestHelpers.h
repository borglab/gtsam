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
 * @author Alex Cunningham
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#pragma once

#include <iostream>
#include <sstream>
#include <string>

#include <gtsam/base/serialization.h>

#include <boost/serialization/serialization.hpp>
#include <boost/filesystem.hpp>


// whether to print the serialized text to stdout
const bool verbose = false;

namespace gtsam {
namespace serializationTestHelpers {

// templated default object creation so we only need to declare one friend (if applicable)
template<class T>
T create() {
  return T();
}

// Creates or empties a folder in the build folder and returns the relative path
inline boost::filesystem::path resetFilesystem(
    boost::filesystem::path folder = "actual") {
  boost::filesystem::remove_all(folder);
  boost::filesystem::create_directory(folder);
  return folder;
}

// Templated round-trip serialization
template<class T>
void roundtrip(const T& input, T& output) {
  std::string serialized = serialize(input);
  if (verbose) std::cout << serialized << std::endl << std::endl;
  deserialize(serialized, output);
}

// Templated round-trip serialization using a file
template<class T>
void roundtripFile(const T& input, T& output) {
  boost::filesystem::path path = resetFilesystem()/"graph.dat";
  serializeToFile(input, path.string());
  deserializeFromFile(path.string(), output);
}

// This version requires equality operator and uses string and file round-trips
template<class T>
bool equality(const T& input = T()) {
  T output = create<T>(), outputf = create<T>();
  roundtrip<T>(input,output);
  roundtripFile<T>(input,outputf);
  return (input==output) && (input==outputf);
}

// This version requires Testable
template<class T>
bool equalsObj(const T& input = T()) {
  T output = create<T>();
  roundtrip<T>(input,output);
  return assert_equal(input, output);
}

// De-referenced version for pointers, requires equals method
template<class T>
bool equalsDereferenced(const T& input) {
  T output = create<T>();
  roundtrip<T>(input,output);
  return input->equals(*output);
}

// Templated round-trip serialization using XML
template<class T>
void roundtripXML(const T& input, T& output) {
  std::string serialized = serializeXML<T>(input);
  if (verbose) std::cout << serialized << std::endl << std::endl;
  deserializeXML(serialized, output);
}

// Templated round-trip serialization using XML File
template<class T>
void roundtripXMLFile(const T& input, T& output) {
  boost::filesystem::path path = resetFilesystem()/"graph.xml";
  serializeToXMLFile(input, path.string());
  deserializeFromXMLFile(path.string(), output);
}

// This version requires equality operator
template<class T>
bool equalityXML(const T& input = T()) {
  T output = create<T>(), outputf = create<T>();
  roundtripXML<T>(input,output);
  roundtripXMLFile<T>(input,outputf);
  return (input==output) && (input==outputf);
}

// This version requires Testable
template<class T>
bool equalsXML(const T& input = T()) {
  T output = create<T>();
  roundtripXML<T>(input,output);
  return assert_equal(input, output);
}

// This version is for pointers, requires equals method
template<class T>
bool equalsDereferencedXML(const T& input = T()) {
  T output = create<T>();
  roundtripXML<T>(input,output);
  return input->equals(*output);
}

// Templated round-trip serialization using XML
template<class T>
void roundtripBinary(const T& input, T& output) {
  std::string serialized = serializeBinary<T>(input);
  if (verbose) std::cout << serialized << std::endl << std::endl;
  deserializeBinary(serialized, output);
}

// Templated round-trip serialization using Binary file
template<class T>
void roundtripBinaryFile(const T& input, T& output) {
  boost::filesystem::path path = resetFilesystem()/"graph.bin";
  serializeToBinaryFile(input, path.string());
  deserializeFromBinaryFile(path.string(), output);
}

// This version requires equality operator
template<class T>
bool equalityBinary(const T& input = T()) {
  T output = create<T>(), outputf = create<T>();
  roundtripBinary<T>(input,output);
  roundtripBinaryFile<T>(input,outputf);
  return (input==output) && (input==outputf);
}

// This version requires Testable
template<class T>
bool equalsBinary(const T& input = T()) {
  T output = create<T>();
  roundtripBinary<T>(input,output);
  return assert_equal(input, output);
}

// This version is for pointers, requires equals method
template<class T>
bool equalsDereferencedBinary(const T& input = T()) {
  T output = create<T>();
  roundtripBinary<T>(input,output);
  return input->equals(*output);
}

} // \namespace serializationTestHelpers
} // \namespace gtsam

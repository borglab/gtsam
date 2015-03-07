/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file serialization.h
 * @brief Convenience functions for serializing data structures via boost.serialization
 * @author Alex Cunningham
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#pragma once

#include <sstream>
#include <fstream>
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
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

namespace gtsam {

// Serialization directly to strings in compressed format
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

template<class T>
bool serializeToFile(const T& input, const std::string& filename) {
  std::ofstream out_archive_stream(filename.c_str());
  if (!out_archive_stream.is_open())
    return false;
  boost::archive::text_oarchive out_archive(out_archive_stream);
  out_archive << input;
  out_archive_stream.close();
  return true;
}

template<class T>
bool deserializeFromFile(const std::string& filename, T& output) {
  std::ifstream in_archive_stream(filename.c_str());
  if (!in_archive_stream.is_open())
    return false;
  boost::archive::text_iarchive in_archive(in_archive_stream);
  in_archive >> output;
  in_archive_stream.close();
  return true;
}

// Serialization to XML format with named structures
template<class T>
std::string serializeXML(const T& input, const std::string& name="data") {
  std::ostringstream out_archive_stream;
  boost::archive::xml_oarchive out_archive(out_archive_stream);
  out_archive << boost::serialization::make_nvp(name.c_str(), input);
  return out_archive_stream.str();
}

template<class T>
void deserializeXML(const std::string& serialized, T& output, const std::string& name="data") {
  std::istringstream in_archive_stream(serialized);
  boost::archive::xml_iarchive in_archive(in_archive_stream);
  in_archive >> boost::serialization::make_nvp(name.c_str(), output);
}

template<class T>
bool serializeToXMLFile(const T& input, const std::string& filename, const std::string& name="data") {
  std::ofstream out_archive_stream(filename.c_str());
  if (!out_archive_stream.is_open())
    return false;
  boost::archive::xml_oarchive out_archive(out_archive_stream);
  out_archive << boost::serialization::make_nvp(name.c_str(), input);;
  out_archive_stream.close();
  return true;
}

template<class T>
bool deserializeFromXMLFile(const std::string& filename, T& output, const std::string& name="data") {
  std::ifstream in_archive_stream(filename.c_str());
  if (!in_archive_stream.is_open())
    return false;
  boost::archive::xml_iarchive in_archive(in_archive_stream);
  in_archive >> boost::serialization::make_nvp(name.c_str(), output);
  in_archive_stream.close();
  return true;
}

// Serialization to binary format with named structures
template<class T>
std::string serializeBinary(const T& input, const std::string& name="data") {
  std::ostringstream out_archive_stream;
  boost::archive::binary_oarchive out_archive(out_archive_stream);
  out_archive << boost::serialization::make_nvp(name.c_str(), input);
  return out_archive_stream.str();
}

template<class T>
void deserializeBinary(const std::string& serialized, T& output, const std::string& name="data") {
  std::istringstream in_archive_stream(serialized);
  boost::archive::binary_iarchive in_archive(in_archive_stream);
  in_archive >> boost::serialization::make_nvp(name.c_str(), output);
}

template<class T>
bool serializeToBinaryFile(const T& input, const std::string& filename, const std::string& name="data") {
  std::ofstream out_archive_stream(filename.c_str());
  if (!out_archive_stream.is_open())
    return false;
  boost::archive::binary_oarchive out_archive(out_archive_stream);
  out_archive << boost::serialization::make_nvp(name.c_str(), input);
  out_archive_stream.close();
  return true;
}

template<class T>
bool deserializeFromBinaryFile(const std::string& filename, T& output, const std::string& name="data") {
  std::ifstream in_archive_stream(filename.c_str());
  if (!in_archive_stream.is_open())
    return false;
  boost::archive::binary_iarchive in_archive(in_archive_stream);
  in_archive >> boost::serialization::make_nvp(name.c_str(), output);
  in_archive_stream.close();
  return true;
}

} // \namespace gtsam

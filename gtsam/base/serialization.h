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
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#pragma once

#include <Eigen/Core>
#include <fstream>
#include <sstream>
#include <string>

// includes for standard serialization types
#include <boost/serialization/version.hpp>
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
#include <boost/serialization/export.hpp>

// Workaround a bug in GCC >= 7 and C++17
// ref. https://gitlab.com/libeigen/eigen/-/issues/1676
#ifdef __GNUC__
#if __GNUC__ >= 7 && __cplusplus >= 201703L
namespace boost { namespace serialization { struct U; } }
namespace Eigen { namespace internal {
template<> struct traits<boost::serialization::U> {enum {Flags=0};};
} }
#endif
#endif

namespace gtsam {

/** @name Standard serialization
 *  Serialization in default compressed format
 */
///@{
/// serializes to a stream
template <class T>
void serializeToStream(const T& input, std::ostream& out_archive_stream) {
  boost::archive::text_oarchive out_archive(out_archive_stream);
  out_archive << input;
}

/// deserializes from a stream
template <class T>
void deserializeFromStream(std::istream& in_archive_stream, T& output) {
  boost::archive::text_iarchive in_archive(in_archive_stream);
  in_archive >> output;
}

/// serializes to a string
template <class T>
std::string serializeToString(const T& input) {
  std::ostringstream out_archive_stream;
  serializeToStream(input, out_archive_stream);
  return out_archive_stream.str();
}

/// deserializes from a string
template <class T>
void deserializeFromString(const std::string& serialized, T& output) {
  std::istringstream in_archive_stream(serialized);
  deserializeFromStream(in_archive_stream, output);
}

/// serializes to a file
template <class T>
bool serializeToFile(const T& input, const std::string& filename) {
  std::ofstream out_archive_stream(filename.c_str());
  if (!out_archive_stream.is_open()) return false;
  serializeToStream(input, out_archive_stream);
  out_archive_stream.close();
  return true;
}

/// deserializes from a file
template <class T>
bool deserializeFromFile(const std::string& filename, T& output) {
  std::ifstream in_archive_stream(filename.c_str());
  if (!in_archive_stream.is_open()) return false;
  deserializeFromStream(in_archive_stream, output);
  in_archive_stream.close();
  return true;
}

/// serializes to a string
template <class T>
std::string serialize(const T& input) {
  return serializeToString(input);
}

/// deserializes from a string
template <class T>
void deserialize(const std::string& serialized, T& output) {
  deserializeFromString(serialized, output);
}
///@}

/** @name XML Serialization
 *  Serialization to XML format with named structures
 */
///@{
/// serializes to a stream in XML
template <class T>
void serializeToXMLStream(const T& input, std::ostream& out_archive_stream,
                  const std::string& name = "data") {
  boost::archive::xml_oarchive out_archive(out_archive_stream);
  out_archive << boost::serialization::make_nvp(name.c_str(), input);
}

/// deserializes from a stream in XML
template <class T>
void deserializeFromXMLStream(std::istream& in_archive_stream, T& output,
                    const std::string& name = "data") {
  boost::archive::xml_iarchive in_archive(in_archive_stream);
  in_archive >> boost::serialization::make_nvp(name.c_str(), output);
}

/// serializes to a string in XML
template <class T>
std::string serializeToXMLString(const T& input,
                         const std::string& name = "data") {
  std::ostringstream out_archive_stream;
  serializeToXMLStream(input, out_archive_stream, name);
  return out_archive_stream.str();
}

/// deserializes from a string in XML
template <class T>
void deserializeFromXMLString(const std::string& serialized, T& output,
                    const std::string& name = "data") {
  std::istringstream in_archive_stream(serialized);
  deserializeFromXMLStream(in_archive_stream, output, name);
}

/// serializes to an XML file
template <class T>
bool serializeToXMLFile(const T& input, const std::string& filename,
                        const std::string& name = "data") {
  std::ofstream out_archive_stream(filename.c_str());
  if (!out_archive_stream.is_open()) return false;
  serializeToXMLStream(input, out_archive_stream, name);
  out_archive_stream.close();
  return true;
}

/// deserializes from an XML file
template <class T>
bool deserializeFromXMLFile(const std::string& filename, T& output,
                            const std::string& name = "data") {
  std::ifstream in_archive_stream(filename.c_str());
  if (!in_archive_stream.is_open()) return false;
  deserializeFromXMLStream(in_archive_stream, output, name);
  in_archive_stream.close();
  return true;
}

/// serializes to a string in XML
template <class T>
std::string serializeXML(const T& input,
                         const std::string& name = "data") {
  return serializeToXMLString(input, name);
}

/// deserializes from a string in XML
template <class T>
void deserializeXML(const std::string& serialized, T& output,
                    const std::string& name = "data") {
  deserializeFromXMLString(serialized, output, name);
}
///@}

/** @name Binary Serialization
 *  Serialization to binary format with named structures
 */
///@{
/// serializes to a stream in binary
template <class T>
void serializeToBinaryStream(const T& input, std::ostream& out_archive_stream,
                     const std::string& name = "data") {
  boost::archive::binary_oarchive out_archive(out_archive_stream);
  out_archive << boost::serialization::make_nvp(name.c_str(), input);
}

/// deserializes from a stream in binary
template <class T>
void deserializeFromBinaryStream(std::istream& in_archive_stream, T& output,
                       const std::string& name = "data") {
  boost::archive::binary_iarchive in_archive(in_archive_stream);
  in_archive >> boost::serialization::make_nvp(name.c_str(), output);
}

/// serializes to a string in binary
template <class T>
std::string serializeToBinaryString(const T& input,
                            const std::string& name = "data") {
  std::ostringstream out_archive_stream;
  serializeToBinaryStream(input, out_archive_stream, name);
  return out_archive_stream.str();
}

/// deserializes from a string in binary
template <class T>
void deserializeFromBinaryString(const std::string& serialized, T& output,
                       const std::string& name = "data") {
  std::istringstream in_archive_stream(serialized);
  deserializeFromBinaryStream(in_archive_stream, output, name);
}

/// serializes to a binary file
template <class T>
bool serializeToBinaryFile(const T& input, const std::string& filename,
                           const std::string& name = "data") {
  std::ofstream out_archive_stream(filename.c_str());
  if (!out_archive_stream.is_open()) return false;
  serializeToBinaryStream(input, out_archive_stream, name);
  out_archive_stream.close();
  return true;
}

/// deserializes from a binary file
template <class T>
bool deserializeFromBinaryFile(const std::string& filename, T& output,
                               const std::string& name = "data") {
  std::ifstream in_archive_stream(filename.c_str());
  if (!in_archive_stream.is_open()) return false;
  deserializeFromBinaryStream(in_archive_stream, output, name);
  in_archive_stream.close();
  return true;
}

/// serializes to a string in binary
template <class T>
std::string serializeBinary(const T& input,
                            const std::string& name = "data") {
  return serializeToBinaryString(input, name);
}

/// deserializes from a string in binary
template <class T>
void deserializeBinary(const std::string& serialized, T& output,
                       const std::string& name = "data") {
  deserializeFromBinaryString(serialized, output, name);
}
///@}

}  // namespace gtsam
#endif
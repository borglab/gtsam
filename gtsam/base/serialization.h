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
#include <cereal/types/map.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/optional.hpp>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

#include <boost/optional.hpp>

namespace cereal {
  //! Saving for std::optional
  template <class Archive, typename T> inline
  void CEREAL_SAVE_FUNCTION_NAME(Archive& ar, const boost::optional<T>& optional)
  {
    if(!optional) {
      ar(CEREAL_NVP_("nullopt", true));
    } else {
      ar(CEREAL_NVP_("nullopt", false),
         CEREAL_NVP_("data", *optional));
    }
  }

  //! Loading for std::optional
  template <class Archive, typename T> inline
  void CEREAL_LOAD_FUNCTION_NAME(Archive& ar, boost::optional<T>& optional)
  {
    bool nullopt;
    ar(CEREAL_NVP_("nullopt", nullopt));

    if (nullopt) {
      optional = boost::none;
    } else {
      T value;
      ar(CEREAL_NVP_("data", value));
      optional = std::move(value);
    }
  }

  template<class Archive, class F, class S>
  void save(Archive& ar, const std::pair<F, S>& pair)
  {
    ar(pair.first, pair.second);
  }

  template<class Archive, class F, class S>
  void load(Archive& ar, std::pair<F, S>& pair)
  {
    ar(pair.first, pair.second);
  }

  template <class Archive, class F, class S>
  struct specialize<Archive, std::pair<F, S>, cereal::specialization::non_member_load_save> {};
} // namespace cereal

namespace gtsam {

/** @name Standard serialization
 *  Serialization in default compressed format
 */
///@{
/// serializes to a stream
template <class T>
void serializeToStream(const T& input, std::ostream& out_archive_stream) {
  cereal::JSONOutputArchive out_archive(out_archive_stream);
  out_archive << input;
}

/// deserializes from a stream
template <class T>
void deserializeFromStream(std::istream& in_archive_stream, T& output) {
  cereal::JSONInputArchive in_archive(in_archive_stream);
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
  cereal::XMLOutputArchive out_archive(out_archive_stream);
  out_archive << cereal::make_nvp(name.c_str(), input);
}

/// deserializes from a stream in XML
template <class T>
void deserializeFromXMLStream(std::istream& in_archive_stream, T& output,
                    const std::string& name = "data") {
  cereal::XMLInputArchive in_archive(in_archive_stream);
  in_archive >> cereal::make_nvp(name.c_str(), output);
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
  cereal::BinaryOutputArchive out_archive(out_archive_stream);
  out_archive << cereal::make_nvp(name.c_str(), input);
}

/// deserializes from a stream in binary
template <class T>
void deserializeFromBinaryStream(std::istream& in_archive_stream, T& output,
                       const std::string& name = "data") {
  cereal::BinaryInputArchive in_archive(in_archive_stream);
  in_archive >> cereal::make_nvp(name.c_str(), output);
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

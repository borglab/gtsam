/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     types.cpp
 * @brief    Functions for handling type information
 * @author   Varun Agrawal
 * @date     May 18, 2020
 * @addtogroup base
 */

#include <gtsam/base/types.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <DbgHelp.h>
#endif

#ifdef __GNUG__
#include <cstdlib>
#include <cxxabi.h>
#include <string>
#endif

namespace gtsam {

/// Pretty print Value type name
std::string demangle(const char* name) {
  // by default set to the original mangled name
  std::string demangled_name = std::string(name);

#ifdef __GNUG__
  // g++ version of demangle
  char* demangled = nullptr;
  int status = -1; // some arbitrary value to eliminate the compiler warning
  demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status),
  
  demangled_name = (status == 0) ? std::string(demangled) : std::string(name);

  std::free(demangled);

#elif _WIN32
    char undecorated_name[1024];
    
    if (UnDecorateSymbolName(
        name, undecorated_name, sizeof(undecorated_name),
        UNDNAME_COMPLETE))
    {
      // successful conversion, take value from: undecorated_name			                      
      demangled_name = std::string(undecorated_name);               
    }
    // else keep using mangled name
#endif

  return demangled_name;
}

std::string GTDKeyFormatter(const Key &key) {
  constexpr size_t kMax_uchar_ = std::numeric_limits<uint8_t>::max();
  constexpr size_t key_bits = sizeof(gtsam::Key) * 8;
  constexpr size_t ch1_bits = sizeof(uint8_t) * 8;
  constexpr size_t ch2_bits = sizeof(uint8_t) * 8;
  constexpr size_t link_bits = sizeof(uint8_t) * 8;
  constexpr size_t joint_bits = sizeof(uint8_t) * 8;
  constexpr size_t time_bits =
      key_bits - ch1_bits - ch2_bits - link_bits - joint_bits;

  constexpr gtsam::Key ch1_mask = gtsam::Key(kMax_uchar_)
                                  << (key_bits - ch1_bits);
  constexpr gtsam::Key ch2_mask = gtsam::Key(kMax_uchar_)
                                  << (key_bits - ch1_bits - ch2_bits);
  constexpr gtsam::Key link_mask = gtsam::Key(kMax_uchar_)
                                   << (time_bits + joint_bits);
  constexpr gtsam::Key joint_mask = gtsam::Key(kMax_uchar_) << time_bits;
  constexpr gtsam::Key time_mask =
      ~(ch1_mask | ch2_mask | link_mask | joint_mask);

  uint8_t c1_, c2_, link_idx_, joint_idx_;
  uint64_t t_;

  c1_ = (uint8_t)((key & ch1_mask) >> (key_bits - ch1_bits));
  c2_ = (uint8_t)((key & ch2_mask) >> (key_bits - ch1_bits - ch2_bits));
  link_idx_ = (uint8_t)((key & link_mask) >> (time_bits + joint_bits));
  joint_idx_ = (uint8_t)((key & joint_mask) >> time_bits);
  t_ = key & time_mask;

  std::string s = "";
  if (c1_ != 0) {
    s += c1_;
  }
  if (c2_ != 0) {
    s += c2_;
  }
  if (link_idx_ != kMax_uchar_) {
    s += "[" + std::to_string((int)(link_idx_)) + "]";
  }
  if (joint_idx_ != kMax_uchar_) {
    s += "(" + std::to_string((int)(joint_idx_)) + ")";
  }
  s += std::to_string(t_);
  return s;
}

} /* namespace gtsam */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     ThreadSafeException.h
 * @brief    Base exception type that uses tbb_allocator if GTSAM is compiled with TBB
 * @author   Richard Roberts
 * @date     Aug 21, 2010
 * @ingroup base
 */

#pragma once

#include <gtsam/config.h> // for GTSAM_USE_TBB

#include <boost/optional/optional.hpp>
#include <gtsam/dllexport.h>
#include <string>
#include <typeinfo>
#include <exception>

#ifdef GTSAM_USE_TBB
#include <tbb/tbb_allocator.h>
#include <tbb/scalable_allocator.h>
#include <iostream>
#endif

namespace gtsam {

/// Base exception type that uses tbb_allocator if GTSAM is compiled with TBB.
template<class DERIVED>
class ThreadsafeException:
public std::exception
{
private:
  typedef std::exception Base;
#ifdef GTSAM_USE_TBB
protected:
  typedef std::basic_string<char, std::char_traits<char>,
      tbb::tbb_allocator<char> > String;
#else
protected:
  typedef std::string String;
#endif

protected:
  bool dynamic_; ///< Whether this object was moved
  mutable boost::optional<String> description_; ///< Optional description

  /// Default constructor is protected - may only be created from derived classes
  ThreadsafeException() :
      dynamic_(false) {
  }

  /// Copy constructor is protected - may only be created from derived classes
  ThreadsafeException(const ThreadsafeException& other) :
      Base(other), dynamic_(false) {
  }

  /// Construct with description string
  ThreadsafeException(const std::string& description) :
      dynamic_(false), description_(
          String(description.begin(), description.end())) {
  }

  /// Default destructor doesn't have the noexcept
  ~ThreadsafeException() noexcept override {
  }

public:
  const char* what() const noexcept override {
    return description_ ? description_->c_str() : "";
  }
};

/// Thread-safe runtime error exception
class GTSAM_EXPORT RuntimeErrorThreadsafe: public ThreadsafeException<RuntimeErrorThreadsafe> {
public:
  /// Construct with a string describing the exception
  RuntimeErrorThreadsafe(const std::string& description) :
      ThreadsafeException<RuntimeErrorThreadsafe>(description) {
  }
};

/// Thread-safe out of range exception
class OutOfRangeThreadsafe: public ThreadsafeException<OutOfRangeThreadsafe> {
public:
  /// Construct with a string describing the exception
  OutOfRangeThreadsafe(const std::string& description) :
      ThreadsafeException<OutOfRangeThreadsafe>(description) {
  }
};

/// Thread-safe invalid argument exception
class InvalidArgumentThreadsafe: public ThreadsafeException<
    InvalidArgumentThreadsafe> {
public:
  /// Construct with a string describing the exception
  InvalidArgumentThreadsafe(const std::string& description) :
      ThreadsafeException<InvalidArgumentThreadsafe>(description) {
  }
};

/// Indicate Cholesky factorization failure
class CholeskyFailed : public gtsam::ThreadsafeException<CholeskyFailed>
{
public:
  CholeskyFailed() noexcept {}
  ~CholeskyFailed() noexcept override {}
};

} // namespace gtsam

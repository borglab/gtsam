/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   IterativeSolver.h
 * @brief  Some support classes for iterative solvers
 * @date   2010
 * @author Yong-Dian Jian
 */

#pragma once

#include <gtsam/inference/Ordering.h>
#include <gtsam/base/Vector.h>

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

#include <iosfwd>
#include <string>
#include <map>

namespace gtsam {

// Forward declarations
struct KeyInfoEntry;
class KeyInfo;
class GaussianFactorGraph;
class Values;
class VectorValues;

/**
 * parameters for iterative linear solvers
 */
class IterativeOptimizationParameters {

public:

  typedef boost::shared_ptr<IterativeOptimizationParameters> shared_ptr;
  enum Verbosity {
    SILENT = 0, COMPLEXITY, ERROR
  } verbosity_;

public:

  IterativeOptimizationParameters(Verbosity v = SILENT) :
      verbosity_(v) {
  }

  virtual ~IterativeOptimizationParameters() {
  }

  /* utility */
  inline Verbosity verbosity() const {
    return verbosity_;
  }
  GTSAM_EXPORT std::string getVerbosity() const;
  GTSAM_EXPORT void setVerbosity(const std::string &s);

  /* matlab interface */
  GTSAM_EXPORT void print() const;

  /* virtual print function */
  GTSAM_EXPORT virtual void print(std::ostream &os) const;

  /* for serialization */
  friend std::ostream& operator<<(std::ostream &os,
      const IterativeOptimizationParameters &p);

  GTSAM_EXPORT static Verbosity verbosityTranslator(const std::string &s);
  GTSAM_EXPORT static std::string verbosityTranslator(Verbosity v);
};

/**
 * Base class for Iterative Solvers like SubgraphSolver
 */
class IterativeSolver {
public:
  typedef boost::shared_ptr<IterativeSolver> shared_ptr;
  IterativeSolver() {
  }
  virtual ~IterativeSolver() {
  }

  /* interface to the nonlinear optimizer, without metadata, damping and initial estimate */
  GTSAM_EXPORT VectorValues optimize(const GaussianFactorGraph &gfg,
      boost::optional<const KeyInfo&> = boost::none,
      boost::optional<const std::map<Key, Vector>&> lambda = boost::none);

  /* interface to the nonlinear optimizer, without initial estimate */
  GTSAM_EXPORT VectorValues optimize(const GaussianFactorGraph &gfg, const KeyInfo &keyInfo,
      const std::map<Key, Vector> &lambda);

  /* interface to the nonlinear optimizer that the subclasses have to implement */
  virtual VectorValues optimize(const GaussianFactorGraph &gfg,
      const KeyInfo &keyInfo, const std::map<Key, Vector> &lambda,
      const VectorValues &initial) = 0;

};

/**
 * Handy data structure for iterative solvers
 * key to (index, dimension, start)
 */
struct GTSAM_EXPORT KeyInfoEntry {
  size_t index, dim, start;
  KeyInfoEntry() {
  }
  KeyInfoEntry(size_t idx, size_t d, Key start) :
      index(idx), dim(d), start(start) {
  }
};

/**
 * Handy data structure for iterative solvers
 */
class GTSAM_EXPORT KeyInfo: public std::map<Key, KeyInfoEntry> {

public:

  typedef std::map<Key, KeyInfoEntry> Base;

protected:

  Ordering ordering_;
  size_t numCols_;

  void initialize(const GaussianFactorGraph &fg);

public:

  /// Default Constructor
  KeyInfo() :
      numCols_(0) {
  }

  /// Construct from Gaussian factor graph, use "Natural" ordering
  KeyInfo(const GaussianFactorGraph &fg);

  /// Construct from Gaussian factor graph and a given ordering
  KeyInfo(const GaussianFactorGraph &fg, const Ordering &ordering);

  /// Return the total number of columns (scalar variables = sum of dimensions)
  inline size_t numCols() const {
    return numCols_;
  }

  /// Return the ordering
  inline const Ordering & ordering() const {
    return ordering_;
  }

  /// Return a vector of dimensions ordered by ordering()
  std::vector<size_t> colSpec() const;

  /// Return VectorValues with zeros, of correct dimension
  VectorValues x0() const;

  /// Return zero Vector of correct dimension
  Vector x0vector() const;

};

} // \ namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/global_includes.h>
#include <gtsam/inference/Ordering.h>
#include <boost/tuple/tuple.hpp>
#include <boost/optional/optional.hpp>
#include <boost/none.hpp>
#include <iosfwd>
#include <map>
#include <string>
#include <vector>

namespace gtsam {

  // Forward declarations
  class KeyInfo;
  class KeyInfoEntry;
  class GaussianFactorGraph;
  class Values;
  class VectorValues;

  /************************************************************************************/
  /**
   * parameters for iterative linear solvers
   */
  class GTSAM_EXPORT IterativeOptimizationParameters {

  public:

    typedef boost::shared_ptr<IterativeOptimizationParameters> shared_ptr;
    enum Verbosity { SILENT = 0, COMPLEXITY, ERROR } verbosity_;

  public:

    IterativeOptimizationParameters(Verbosity v = SILENT)
      : verbosity_(v) {}

    virtual ~IterativeOptimizationParameters() {}

    /* utility */
    inline Verbosity verbosity() const { return verbosity_; }
    std::string getVerbosity() const;
    void setVerbosity(const std::string &s) ;

    /* matlab interface */
    void print() const ;

    /* virtual print function */
    virtual void print(std::ostream &os) const ;

    /* for serialization */
    friend std::ostream& operator<<(std::ostream &os, const IterativeOptimizationParameters &p);

    static Verbosity verbosityTranslator(const std::string &s);
    static std::string verbosityTranslator(Verbosity v);
  };

  /************************************************************************************/
  class GTSAM_EXPORT IterativeSolver {
  public:
    typedef boost::shared_ptr<IterativeSolver> shared_ptr;
    IterativeSolver() {}
    virtual ~IterativeSolver() {}

    /* interface to the nonlinear optimizer, without metadata, damping and initial estimate */
    VectorValues optimize (
      const GaussianFactorGraph &gfg,
      boost::optional<const KeyInfo&> = boost::none,
      boost::optional<const std::map<Key, Vector>&> lambda = boost::none
    );

    /* interface to the nonlinear optimizer, without initial estimate */
    VectorValues optimize (
      const GaussianFactorGraph &gfg,
      const KeyInfo &keyInfo,
      const std::map<Key, Vector> &lambda
    );

    /* interface to the nonlinear optimizer that the subclasses have to implement */
    virtual VectorValues optimize (
      const GaussianFactorGraph &gfg,
      const KeyInfo &keyInfo,
      const std::map<Key, Vector> &lambda,
      const VectorValues &initial
    ) = 0;

  };

  /************************************************************************************/
  /* Handy data structure for iterative solvers
   * key to (index, dimension, colstart) */
  class GTSAM_EXPORT KeyInfoEntry : public boost::tuple<size_t, size_t, size_t> {
  public:
    typedef boost::tuple<Key,size_t,Key> Base;
    KeyInfoEntry(){}
    KeyInfoEntry(size_t idx, size_t d, Key start) : Base(idx, d, start) {}
    const size_t index() const { return this->get<0>(); }
    const size_t dim() const { return this->get<1>(); }
    const size_t colstart() const { return this->get<2>(); }
  };

  /************************************************************************************/
  class GTSAM_EXPORT KeyInfo : public std::map<Key, KeyInfoEntry> {
  public:
    typedef std::map<Key, KeyInfoEntry> Base;
    KeyInfo() : numCols_(0) {}
    KeyInfo(const GaussianFactorGraph &fg);
    KeyInfo(const GaussianFactorGraph &fg, const Ordering &ordering);

    std::vector<size_t> colSpec() const ;
    VectorValues x0() const;
    Vector x0vector() const;

    inline size_t numCols() const { return numCols_; }
    inline const Ordering & ordering() const { return ordering_; }

  protected:

    void initialize(const GaussianFactorGraph &fg);

    Ordering ordering_;
    size_t numCols_;

  };


}

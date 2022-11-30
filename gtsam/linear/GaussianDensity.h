/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianDensity.h
 * @brief   A Gaussian Density
 * @author  Frank Dellaert
 * @date    Jan 21, 2012
 */

// \callgraph
#pragma once

#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

  /**
  * A GaussianDensity is a GaussianConditional without parents.
  * The negative log-probability is given by \f$ |Rx - d|^2 \f$
  * with \f$ \Lambda = \Sigma^{-1} = R^T R \f$ and \f$ \mu = R^{-1} d \f$
  * @ingroup linear
  */
  class GTSAM_EXPORT GaussianDensity : public GaussianConditional {

  public:

    typedef boost::shared_ptr<GaussianDensity> shared_ptr;

    /// default constructor needed for serialization
    GaussianDensity() :
      GaussianConditional() {
    }

    /// Copy constructor from GaussianConditional
    GaussianDensity(const GaussianConditional& conditional) :
      GaussianConditional(conditional) {
        if(conditional.nrParents() != 0)
          throw std::invalid_argument("GaussianDensity can only be created from a conditional with no parents");
    }

    /// constructor using d, R
    GaussianDensity(Key key, const Vector& d, const Matrix& R, const SharedDiagonal& noiseModel = SharedDiagonal()) :
      GaussianConditional(key, d, R, noiseModel) {}

    /// Construct using a mean and standard deviation
    static GaussianDensity FromMeanAndStddev(Key key, const Vector& mean,
                                             double sigma);

    /// print
    void print(const std::string& = "GaussianDensity",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

    /// Mean \f$ \mu = R^{-1} d \f$
    Vector mean() const;

    /// Covariance matrix \f$ \Sigma = (R^T R)^{-1} \f$
    Matrix covariance() const;

  };
  // GaussianDensity

}// gtsam

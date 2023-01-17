/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  AntiFactor.h
 *  @author Stephen Williams
 **/
#pragma once

#include <ostream>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>

namespace gtsam {

  /**
   * A class for downdating an existing factor from a graph. The AntiFactor returns the same
   * linearized Hessian matrices of the original factor, but with the opposite sign. This effectively
   * cancels out any affects of the original factor during optimization."
   * @ingroup slam
   */
  class AntiFactor: public NonlinearFactor {

  private:

    typedef AntiFactor This;
    typedef NonlinearFactor Base;
    typedef NonlinearFactor::shared_ptr sharedFactor;

    sharedFactor factor_;

  public:

    // shorthand for a smart pointer to a factor
    typedef std::shared_ptr<AntiFactor> shared_ptr;

    /** default constructor - only use for serialization */
    AntiFactor() {}

    /** constructor - Creates the equivalent AntiFactor from an existing factor */
    AntiFactor(NonlinearFactor::shared_ptr factor) : Base(factor->keys()), factor_(factor) {}

    ~AntiFactor() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "AntiFactor version of:" << std::endl;
      factor_->print(s, keyFormatter);
    }

    /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) && this->factor_->equals(*e->factor_, tol);
    }

    /** implement functions needed to derive from Factor */

    /**
     * Calculate the error of the factor
     * For the AntiFactor, this will have the same magnitude of the original factor,
     * but the opposite sign.
     */
    double error(const Values& c) const override { return -factor_->error(c); }

    /** get the dimension of the factor (same as the original factor) */
    size_t dim() const override { return factor_->dim(); }

    /**
     * Checks whether this factor should be used based on a set of values.
     * The AntiFactor will have the same 'active' profile as the original factor.
     */
    bool active(const Values& c) const override { return factor_->active(c); }

    /**
     * Linearize to a GaussianFactor. The AntiFactor always returns a Hessian Factor
     * with the same Hessian matrices as the original factor (even if the original factor
     * returns a Jacobian instead of a full Hessian), but with the opposite sign. This
     * effectively cancels the effect of the original factor on the factor graph.
     */
    std::shared_ptr<GaussianFactor> linearize(const Values& c) const override {

      // Generate the linearized factor from the contained nonlinear factor
      GaussianFactor::shared_ptr gaussianFactor = factor_->linearize(c);

      // return the negated version of the factor
      return gaussianFactor->negate();
    }


  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("AntiFactor",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(factor_);
    }
  }; // \class AntiFactor

} /// namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphPreconditioner.h
 * @date Dec 31, 2009
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/global_includes.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/Errors.h>

#include <boost/shared_ptr.hpp>

namespace gtsam {

  // Forward declarations
  class GaussianBayesNet;
  class GaussianFactorGraph;
  class VectorValues;

  /**
   * Subgraph conditioner class, as explained in the RSS 2010 submission.
   * Starting with a graph A*x=b, we split it in two systems A1*x=b1 and A2*x=b2
   * We solve R1*x=c1, and make the substitution y=R1*x-c1.
   * To use the class, give the Bayes Net R1*x=c1 and Graph A2*x=b2.
   * Then solve for yhat using CG, and solve for xhat = system.x(yhat).
   */
  class GTSAM_EXPORT SubgraphPreconditioner {

  public:
    typedef boost::shared_ptr<SubgraphPreconditioner> shared_ptr;
    typedef boost::shared_ptr<const GaussianBayesNet> sharedBayesNet;
    typedef boost::shared_ptr<const GaussianFactorGraph> sharedFG;
    typedef boost::shared_ptr<const VectorValues> sharedValues;
    typedef boost::shared_ptr<const Errors> sharedErrors;

  private:
    sharedFG Ab2_;
    sharedBayesNet Rc1_;
    sharedValues xbar_;  ///< A1 \ b1
    sharedErrors b2bar_; ///< A2*xbar - b2

  public:

    SubgraphPreconditioner();

    /**
     * Constructor
     * @param Ab2: the Graph A2*x=b2
     * @param Rc1: the Bayes Net R1*x=c1
     * @param xbar: the solution to R1*x=c1
     */
    SubgraphPreconditioner(const sharedFG& Ab2, const sharedBayesNet& Rc1, const sharedValues& xbar);

    /** print the object */
    void print(const std::string& s = "SubgraphPreconditioner") const;

    /** Access Ab2 */
    const sharedFG& Ab2() const { return Ab2_; }

    /** Access Rc1 */
    const sharedBayesNet& Rc1() const { return Rc1_; }

    /** Access b2bar */
    const sharedErrors b2bar() const { return b2bar_; }

    /**
     * Add zero-mean i.i.d. Gaussian prior terms to each variable
     * @param sigma Standard deviation of Gaussian
     */
//  SubgraphPreconditioner add_priors(double sigma) const;

    /* x = xbar + inv(R1)*y */
    VectorValues x(const VectorValues& y) const;

    /* A zero VectorValues with the structure of xbar */
    VectorValues zero() const {
      VectorValues V(VectorValues::Zero(*xbar_));
      return V ;
    }

    /**
     * Add constraint part of the error only
     * y += alpha*inv(R1')*A2'*e2
     * Takes a range indicating e2 !!!!
     */
    void transposeMultiplyAdd2(double alpha, Errors::const_iterator begin,
        Errors::const_iterator end, VectorValues& y) const;

    /* error, given y */
    double error(const VectorValues& y) const;

    /** gradient = y + inv(R1')*A2'*(A2*inv(R1)*y-b2bar) */
    VectorValues gradient(const VectorValues& y) const;

    /** Apply operator A */
    Errors operator*(const VectorValues& y) const;

    /** Apply operator A in place: needs e allocated already */
    void multiplyInPlace(const VectorValues& y, Errors& e) const;

    /** Apply operator A' */
    VectorValues operator^(const Errors& e) const;

    /**
    * Add A'*e to y
    *  y += alpha*A'*[e1;e2] = [alpha*e1; alpha*inv(R1')*A2'*e2]
    */
    void transposeMultiplyAdd(double alpha, const Errors& e, VectorValues& y) const;
  };

} // namespace gtsam

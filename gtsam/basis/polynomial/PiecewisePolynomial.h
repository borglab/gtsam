#pragma once

#include <gtsam/basis/polynomial/kernels.h>

namespace gtsam {


/**
 * A Piecewise-defined Polynomial in 1d
 * mostly used for low-order approximations to probability distributions
 * automatically computes derivatives
 * 
 */
template<size_t O, size_t P>
class PiecewisePolynomial : public KernelBase
{
public:
  inline constexpr static auto order = O;
  inline constexpr static auto pieces = P;

protected:

  struct param_s{
    std::array<std::array<double, order+1>, pieces > coefficients;
    std::array<double, pieces+1> intervals;
    double center;
  };
  const param_s params;


public:
  PiecewisePolynomial(param_s init_list):params(init_list){}
  /** returns the centre of the distribution */
  double getCenter() const override
  {
    return params.center;
  }
  /** returns the lowest value of t where this polynomial has non-constant return */
  double getBeginning() const override
  {
    return params.intervals[0];
  }
  /** returns the highest value of t where this polynomial has non-constant return */
  double getEnd() const override
  {
    return params.intervals[pieces];
  }
  /** returns the number of times this function can be differentiated and still return something useful */
  size_t getValidDerivatives() const override
  {
    return order;
  }

  /** returns the number of pieces this polynomial is defined by, mostly use for testing. */
  std::array<double, pieces+1> getIntervals() const {
    return params.intervals;
  }

  /**
   * evaluates the polynomial at t
   * @param t  the evaluation point
   * @param H  the first derivative at t
   */
  double evaluate(const double& t, OptionalJacobian<1, 1>  H = {}) const override
  {
    return evaluateDerivative(0, t, H);
  }

  /**
   * evaluates the polynomial or its derivatives at t
   * @param derivative the derivative to evaluate
   * (0th derivative is the same as evaluate(t,H))
   * @param t  the evaluation point
   * @param H  the next derivative at t
   */
  double evaluateDerivative(size_t derivative, double t, OptionalJacobian<1, 1>  H = {}) const override
  {
    // bounds check
    if(t < params.intervals[0])
    {
      t = params.intervals[0];
    }
    if(t > params.intervals[pieces])
    {
      t = params.intervals[pieces];
    }

    // locate the interval
    for(size_t p = 0; p < pieces; p++)
    {
      if(t<=params.intervals[p+1])
      {
        double t_power = 1; // powers of t
        double coeff = 0; // kernel coefficient result
        // iterate through the order of the polynomial
        for(size_t o = derivative; o < order+1; o++)
        {
          // power rule on repeated derivatives is halfway factorial
          // o! / (o-derivative)!
          size_t power_rule = 1;
          for(size_t d=0; d<derivative; d++){
            power_rule *= (o-d);
          }
          coeff += power_rule * t_power * params.coefficients[p][o];
          //std::cout << "c += " << t_power << " * " << params.coefficients[p][o] << std::endl;
          t_power *= t;
        }
        if(H) *H << evaluateDerivative(derivative+1, t, {});
        return coeff;
      }
    }
    // dead code
    if(H) *H << 0;
    return 0;
  }

};



} // namespace gtsam

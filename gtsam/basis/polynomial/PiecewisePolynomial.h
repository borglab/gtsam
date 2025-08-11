#pragma once

#include <gtsam/basis/polynomial/kernels.h>

namespace gtsam {



template<size_t O, size_t P>
class piecewise_polynomial : public kernel_base
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
  piecewise_polynomial(param_s init_list):params(init_list){}

  double get_center() const override
  {
    return params.center;
  }
  double get_beginning() const override
  {
    return params.intervals[0];
  }
  double get_end() const override
  {
    return params.intervals[pieces];
  }
  size_t get_valid_derivatives() const override
  {
    return order;
  }

  std::array<double, pieces+1> get_intervals() const {
    return params.intervals;
  }

/**
  evaluates the continuous kernel function at time t

  derivative: how many times to differentiate the kernel function
  t: the time to evaluate it relative to the kernel's datum
  H: the jacobian for gtsam (the next derivative, calls this function again.)

 */
  double evaluate(const double& t, OptionalJacobian<1, 1>  H = {}) const override
  {
    return evaluate_d(0, t, H);
  }

  double evaluate_d(size_t derivative, double t, OptionalJacobian<1, 1>  H = {}) const override
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
        if(H) *H << evaluate_d(derivative+1, t, {});
        return coeff;
      }
    }
    // dead code
    if(H) *H << 0;
    return 0;
  }

};



} // namespace gtsam

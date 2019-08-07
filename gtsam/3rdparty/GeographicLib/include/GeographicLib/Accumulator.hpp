/**
 * \file Accumulator.hpp
 * \brief Header for GeographicLib::Accumulator class
 *
 * Copyright (c) Charles Karney (2010-2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_ACCUMULATOR_HPP)
#define GEOGRAPHICLIB_ACCUMULATOR_HPP 1

#include <GeographicLib/Constants.hpp>

namespace GeographicLib {

  /**
   * \brief An accumulator for sums
   *
   * This allows many numbers of floating point type \e T to be added together
   * with twice the normal precision.  Thus if \e T is double, the effective
   * precision of the sum is 106 bits or about 32 decimal places.
   *
   * The implementation follows J. R. Shewchuk,
   * <a href="https://doi.org/10.1007/PL00009321"> Adaptive Precision
   * Floating-Point Arithmetic and Fast Robust Geometric Predicates</a>,
   * Discrete & Computational Geometry 18(3) 305--363 (1997).
   *
   * Approximate timings (summing a vector<double>)
   * - double:               2ns
   * - Accumulator<double>: 23ns
   *
   * In the documentation of the member functions, \e sum stands for the value
   * currently held in the accumulator.
   *
   * Example of use:
   * \include example-Accumulator.cpp
   **********************************************************************/
  template<typename T = Math::real>
  class GEOGRAPHICLIB_EXPORT Accumulator {
  private:
    // _s + _t accumulators for the sum.
    T _s, _t;
    // Same as Math::sum, but requires abs(u) >= abs(v).  This isn't currently
    // used.
    static T fastsum(T u, T v, T& t) {
      GEOGRAPHICLIB_VOLATILE T s = u + v;
      GEOGRAPHICLIB_VOLATILE T vp = s - u;
      t = v - vp;
      return s;
    }
    void Add(T y) {
      // Here's Shewchuk's solution...
      T u;                       // hold exact sum as [s, t, u]
      // Accumulate starting at least significant end
      y  = Math::sum(y, _t,  u);
      _s = Math::sum(y, _s, _t);
      // Start is _s, _t decreasing and non-adjacent.  Sum is now (s + t + u)
      // exactly with s, t, u non-adjacent and in decreasing order (except for
      // possible zeros).  The following code tries to normalize the result.
      // Ideally, we want _s = round(s+t+u) and _u = round(s+t+u - _s).  The
      // following does an approximate job (and maintains the decreasing
      // non-adjacent property).  Here are two "failures" using 3-bit floats:
      //
      // Case 1: _s is not equal to round(s+t+u) -- off by 1 ulp
      // [12, -1] - 8 -> [4, 0, -1] -> [4, -1] = 3 should be [3, 0] = 3
      //
      // Case 2: _s+_t is not as close to s+t+u as it shold be
      // [64, 5] + 4 -> [64, 8, 1] -> [64,  8] = 72 (off by 1)
      //                    should be [80, -7] = 73 (exact)
      //
      // "Fixing" these problems is probably not worth the expense.  The
      // representation inevitably leads to small errors in the accumulated
      // values.  The additional errors illustrated here amount to 1 ulp of the
      // less significant word during each addition to the Accumulator and an
      // additional possible error of 1 ulp in the reported sum.
      //
      // Incidentally, the "ideal" representation described above is not
      // canonical, because _s = round(_s + _t) may not be true.  For example,
      // with 3-bit floats:
      //
      // [128, 16] + 1 -> [160, -16] -- 160 = round(145).
      // But [160, 0] - 16 -> [128, 16] -- 128 = round(144).
      //
      if (_s == 0)              // This implies t == 0,
        _s = u;                 // so result is u
      else
        _t += u;                // otherwise just accumulate u to t.
    }
    T Sum(T y) const {
      Accumulator a(*this);
      a.Add(y);
      return a._s;
    }
  public:
    /**
     * Construct from a \e T.  This is not declared explicit, so that you can
     * write <code>Accumulator<double> a = 5;</code>.
     *
     * @param[in] y set \e sum = \e y.
     **********************************************************************/
    Accumulator(T y = T(0)) : _s(y), _t(0) {
      GEOGRAPHICLIB_STATIC_ASSERT(!std::numeric_limits<T>::is_integer,
                                  "Accumulator type is not floating point");
    }
    /**
     * Set the accumulator to a number.
     *
     * @param[in] y set \e sum = \e y.
     **********************************************************************/
    Accumulator& operator=(T y) { _s = y; _t = 0; return *this; }
    /**
     * Return the value held in the accumulator.
     *
     * @return \e sum.
     **********************************************************************/
    T operator()() const { return _s; }
    /**
     * Return the result of adding a number to \e sum (but don't change \e
     * sum).
     *
     * @param[in] y the number to be added to the sum.
     * @return \e sum + \e y.
     **********************************************************************/
    T operator()(T y) const { return Sum(y); }
    /**
     * Add a number to the accumulator.
     *
     * @param[in] y set \e sum += \e y.
     **********************************************************************/
    Accumulator& operator+=(T y) { Add(y); return *this; }
    /**
     * Subtract a number from the accumulator.
     *
     * @param[in] y set \e sum -= \e y.
     **********************************************************************/
    Accumulator& operator-=(T y) { Add(-y); return *this; }
    /**
     * Multiply accumulator by an integer.  To avoid loss of accuracy, use only
     * integers such that \e n &times; \e T is exactly representable as a \e T
     * (i.e., &plusmn; powers of two).  Use \e n = &minus;1 to negate \e sum.
     *
     * @param[in] n set \e sum *= \e n.
     **********************************************************************/
    Accumulator& operator*=(int n) { _s *= n; _t *= n; return *this; }
    /**
     * Multiply accumulator by a number.  The fma (fused multiply and add)
     * instruction is used (if available) in order to maintain accuracy.
     *
     * @param[in] y set \e sum *= \e y.
     **********************************************************************/
    Accumulator& operator*=(T y) {
      T d = _s; _s *= y;
      d = Math::fma(y, d, -_s); // the error in the first multiplication
      _t = Math::fma(y, _t, d); // add error to the second term
      return *this;
    }
    /**
     * Test equality of an Accumulator with a number.
     **********************************************************************/
    bool operator==(T y) const { return _s == y; }
    /**
     * Test inequality of an Accumulator with a number.
     **********************************************************************/
    bool operator!=(T y) const { return _s != y; }
    /**
     * Less operator on an Accumulator and a number.
     **********************************************************************/
    bool operator<(T y) const { return _s < y; }
    /**
     * Less or equal operator on an Accumulator and a number.
     **********************************************************************/
    bool operator<=(T y) const { return _s <= y; }
    /**
     * Greater operator on an Accumulator and a number.
     **********************************************************************/
    bool operator>(T y) const { return _s > y; }
    /**
     * Greater or equal operator on an Accumulator and a number.
     **********************************************************************/
    bool operator>=(T y) const { return _s >= y; }
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_ACCUMULATOR_HPP

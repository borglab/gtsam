/**
 * \file SphericalEngine.hpp
 * \brief Header for GeographicLib::SphericalEngine class
 *
 * Copyright (c) Charles Karney (2011-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_SPHERICALENGINE_HPP)
#define GEOGRAPHICLIB_SPHERICALENGINE_HPP 1

#include <vector>
#include <istream>
#include <GeographicLib/Constants.hpp>

#if defined(_MSC_VER)
// Squelch warnings about dll vs vector
#  pragma warning (push)
#  pragma warning (disable: 4251)
#endif

namespace GeographicLib {

  class CircularEngine;

  /**
   * \brief The evaluation engine for SphericalHarmonic
   *
   * This serves as the backend to SphericalHarmonic, SphericalHarmonic1, and
   * SphericalHarmonic2.  Typically end-users will not have to access this
   * class directly.
   *
   * See SphericalEngine.cpp for more information on the implementation.
   *
   * Example of use:
   * \include example-SphericalEngine.cpp
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT SphericalEngine {
  private:
    typedef Math::real real;
    // CircularEngine needs access to sqrttable, scale
    friend class CircularEngine;
    // Return the table of the square roots of integers
    static std::vector<real>& sqrttable();
    // An internal scaling of the coefficients to avoid overflow in
    // intermediate calculations.
    static real scale() {
      using std::pow;
      static const real
        // Need extra real because, since C++11, pow(float, int) returns double
        s = real(pow(real(std::numeric_limits<real>::radix),
                     -3 * (std::numeric_limits<real>::max_exponent < (1<<14) ?
                           std::numeric_limits<real>::max_exponent : (1<<14))
                     / 5));
      return s;
    }
    // Move latitudes near the pole off the axis by this amount.
    static real eps() {
      using std::sqrt;
      return std::numeric_limits<real>::epsilon() *
        sqrt(std::numeric_limits<real>::epsilon());
    }
    SphericalEngine();          // Disable constructor
  public:
    /**
     * Supported normalizations for associated Legendre polynomials.
     **********************************************************************/
    enum normalization {
      /**
       * Fully normalized associated Legendre polynomials.  See
       * SphericalHarmonic::FULL for documentation.
       *
       * @hideinitializer
       **********************************************************************/
      FULL = 0,
      /**
       * Schmidt semi-normalized associated Legendre polynomials.  See
       * SphericalHarmonic::SCHMIDT for documentation.
       *
       * @hideinitializer
       **********************************************************************/
      SCHMIDT = 1,
    };

    /**
     * \brief Package up coefficients for SphericalEngine
     *
     * This packages up the \e C, \e S coefficients and information about how
     * the coefficients are stored into a single structure.  This allows a
     * vector of type SphericalEngine::coeff to be passed to
     * SphericalEngine::Value.  This class also includes functions to aid
     * indexing into \e C and \e S.
     *
     * The storage layout of the coefficients is documented in
     * SphericalHarmonic and SphericalHarmonic::SphericalHarmonic.
     **********************************************************************/
    class GEOGRAPHICLIB_EXPORT coeff {
    private:
      int _Nx, _nmx, _mmx;
      std::vector<real>::const_iterator _Cnm;
      std::vector<real>::const_iterator _Snm;
    public:
      /**
       * A default constructor
       **********************************************************************/
      coeff() : _Nx(-1) , _nmx(-1) , _mmx(-1) {}
      /**
       * The general constructor.
       *
       * @param[in] C a vector of coefficients for the cosine terms.
       * @param[in] S a vector of coefficients for the sine terms.
       * @param[in] N the degree giving storage layout for \e C and \e S.
       * @param[in] nmx the maximum degree to be used.
       * @param[in] mmx the maximum order to be used.
       * @exception GeographicErr if \e N, \e nmx, and \e mmx do not satisfy
       *   \e N &ge; \e nmx &ge; \e mmx &ge; &minus;1.
       * @exception GeographicErr if \e C or \e S is not big enough to hold the
       *   coefficients.
       * @exception std::bad_alloc if the memory for the square root table
       *   can't be allocated.
       **********************************************************************/
      coeff(const std::vector<real>& C,
            const std::vector<real>& S,
            int N, int nmx, int mmx)
        : _Nx(N)
        , _nmx(nmx)
        , _mmx(mmx)
        , _Cnm(C.begin())
        , _Snm(S.begin())
      {
        if (!(_Nx >= _nmx && _nmx >= _mmx && _mmx >= -1))
          throw GeographicErr("Bad indices for coeff");
        if (!(index(_nmx, _mmx) < int(C.size()) &&
              index(_nmx, _mmx) < int(S.size()) + (_Nx + 1)))
          throw GeographicErr("Arrays too small in coeff");
        SphericalEngine::RootTable(_nmx);
      }
      /**
       * The constructor for full coefficient vectors.
       *
       * @param[in] C a vector of coefficients for the cosine terms.
       * @param[in] S a vector of coefficients for the sine terms.
       * @param[in] N the maximum degree and order.
       * @exception GeographicErr if \e N does not satisfy \e N &ge; &minus;1.
       * @exception GeographicErr if \e C or \e S is not big enough to hold the
       *   coefficients.
       * @exception std::bad_alloc if the memory for the square root table
       *   can't be allocated.
       **********************************************************************/
      coeff(const std::vector<real>& C,
            const std::vector<real>& S,
            int N)
        : _Nx(N)
        , _nmx(N)
        , _mmx(N)
        , _Cnm(C.begin())
        , _Snm(S.begin())
      {
        if (!(_Nx >= -1))
          throw GeographicErr("Bad indices for coeff");
        if (!(index(_nmx, _mmx) < int(C.size()) &&
              index(_nmx, _mmx) < int(S.size()) + (_Nx + 1)))
          throw GeographicErr("Arrays too small in coeff");
        SphericalEngine::RootTable(_nmx);
      }
      /**
       * @return \e N the degree giving storage layout for \e C and \e S.
       **********************************************************************/
      int N() const { return _Nx; }
      /**
       * @return \e nmx the maximum degree to be used.
       **********************************************************************/
      int nmx() const { return _nmx; }
      /**
       * @return \e mmx the maximum order to be used.
       **********************************************************************/
      int mmx() const { return _mmx; }
      /**
       * The one-dimensional index into \e C and \e S.
       *
       * @param[in] n the degree.
       * @param[in] m the order.
       * @return the one-dimensional index.
       **********************************************************************/
      int index(int n, int m) const
      { return m * _Nx - m * (m - 1) / 2 + n; }
      /**
       * An element of \e C.
       *
       * @param[in] k the one-dimensional index.
       * @return the value of the \e C coefficient.
       **********************************************************************/
      Math::real Cv(int k) const { return *(_Cnm + k); }
      /**
       * An element of \e S.
       *
       * @param[in] k the one-dimensional index.
       * @return the value of the \e S coefficient.
       **********************************************************************/
      Math::real Sv(int k) const { return *(_Snm + (k - (_Nx + 1))); }
      /**
       * An element of \e C with checking.
       *
       * @param[in] k the one-dimensional index.
       * @param[in] n the requested degree.
       * @param[in] m the requested order.
       * @param[in] f a multiplier.
       * @return the value of the \e C coefficient multiplied by \e f in \e n
       *   and \e m are in range else 0.
       **********************************************************************/
      Math::real Cv(int k, int n, int m, real f) const
      { return m > _mmx || n > _nmx ? 0 : *(_Cnm + k) * f; }
      /**
       * An element of \e S with checking.
       *
       * @param[in] k the one-dimensional index.
       * @param[in] n the requested degree.
       * @param[in] m the requested order.
       * @param[in] f a multiplier.
       * @return the value of the \e S coefficient multiplied by \e f in \e n
       *   and \e m are in range else 0.
       **********************************************************************/
      Math::real Sv(int k, int n, int m, real f) const
      { return m > _mmx || n > _nmx ? 0 : *(_Snm + (k - (_Nx + 1))) * f; }

      /**
       * The size of the coefficient vector for the cosine terms.
       *
       * @param[in] N the maximum degree.
       * @param[in] M the maximum order.
       * @return the size of the vector of cosine terms as stored in column
       *   major order.
       **********************************************************************/
      static int Csize(int N, int M)
      { return (M + 1) * (2 * N - M + 2) / 2; }

      /**
       * The size of the coefficient vector for the sine terms.
       *
       * @param[in] N the maximum degree.
       * @param[in] M the maximum order.
       * @return the size of the vector of cosine terms as stored in column
       *   major order.
       **********************************************************************/
      static int Ssize(int N, int M)
      { return Csize(N, M) - (N + 1); }

      /**
       * Load coefficients from a binary stream.
       *
       * @param[in] stream the input stream.
       * @param[out] N The maximum degree of the coefficients.
       * @param[out] M The maximum order of the coefficients.
       * @param[out] C The vector of cosine coefficients.
       * @param[out] S The vector of sine coefficients.
       * @exception GeographicErr if \e N and \e M do not satisfy \e N &ge;
       *   \e M &ge; &minus;1.
       * @exception GeographicErr if there's an error reading the data.
       * @exception std::bad_alloc if the memory for \e C or \e S can't be
       *   allocated.
       *
       * \e N and \e M are read as 4-byte ints.  \e C and \e S are resized to
       * accommodate all the coefficients (with the \e m = 0 coefficients for
       * \e S excluded) and the data for these coefficients read as 8-byte
       * doubles.  The coefficients are stored in column major order.  The
       * bytes in the stream should use little-endian ordering.  IEEE floating
       * point is assumed for the coefficients.
       **********************************************************************/
      static void readcoeffs(std::istream& stream, int& N, int& M,
                             std::vector<real>& C, std::vector<real>& S);
    };

    /**
     * Evaluate a spherical harmonic sum and its gradient.
     *
     * @tparam gradp should the gradient be calculated.
     * @tparam norm the normalization for the associated Legendre polynomials.
     * @tparam L the number of terms in the coefficients.
     * @param[in] c an array of coeff objects.
     * @param[in] f array of coefficient multipliers.  f[0] should be 1.
     * @param[in] x the \e x component of the cartesian position.
     * @param[in] y the \e y component of the cartesian position.
     * @param[in] z the \e z component of the cartesian position.
     * @param[in] a the normalizing radius.
     * @param[out] gradx the \e x component of the gradient.
     * @param[out] grady the \e y component of the gradient.
     * @param[out] gradz the \e z component of the gradient.
     * @result the spherical harmonic sum.
     *
     * See the SphericalHarmonic class for the definition of the sum.  The
     * coefficients used by this function are, for example, c[0].Cv + f[1] *
     * c[1].Cv + ... + f[L&minus;1] * c[L&minus;1].Cv.  (Note that f[0] is \e
     * not used.)  The upper limits on the sum are determined by c[0].nmx() and
     * c[0].mmx(); these limits apply to \e all the components of the
     * coefficients.  The parameters \e gradp, \e norm, and \e L are template
     * parameters, to allow more optimization to be done at compile time.
     *
     * Clenshaw summation is used which permits the evaluation of the sum
     * without the need to allocate temporary arrays.  Thus this function never
     * throws an exception.
     **********************************************************************/
    template<bool gradp, normalization norm, int L>
      static Math::real Value(const coeff c[], const real f[],
                              real x, real y, real z, real a,
                              real& gradx, real& grady, real& gradz);

    /**
     * Create a CircularEngine object
     *
     * @tparam gradp should the gradient be calculated.
     * @tparam norm the normalization for the associated Legendre polynomials.
     * @tparam L the number of terms in the coefficients.
     * @param[in] c an array of coeff objects.
     * @param[in] f array of coefficient multipliers.  f[0] should be 1.
     * @param[in] p the radius of the circle = sqrt(<i>x</i><sup>2</sup> +
     *   <i>y</i><sup>2</sup>).
     * @param[in] z the height of the circle.
     * @param[in] a the normalizing radius.
     * @exception std::bad_alloc if the memory for the CircularEngine can't be
     *   allocated.
     * @result the CircularEngine object.
     *
     * If you need to evaluate the spherical harmonic sum for several points
     * with constant \e f, \e p = sqrt(<i>x</i><sup>2</sup> +
     * <i>y</i><sup>2</sup>), \e z, and \e a, it is more efficient to construct
     * call SphericalEngine::Circle to give a CircularEngine object and then
     * call CircularEngine::operator()() with arguments <i>x</i>/\e p and
     * <i>y</i>/\e p.
     **********************************************************************/
    template<bool gradp, normalization norm, int L>
      static CircularEngine Circle(const coeff c[], const real f[],
                                   real p, real z, real a);
    /**
     * Check that the static table of square roots is big enough and enlarge it
     * if necessary.
     *
     * @param[in] N the maximum degree to be used in SphericalEngine.
     * @exception std::bad_alloc if the memory for the square root table can't
     *   be allocated.
     *
     * Typically, there's no need for an end-user to call this routine, because
     * the constructors for SphericalEngine::coeff do so.  However, since this
     * updates a static table, there's a possible race condition in a
     * multi-threaded environment.  Because this routine does nothing if the
     * table is already large enough, one way to avoid race conditions is to
     * call this routine at program start up (when it's still single threaded),
     * supplying the largest degree that your program will use.  E.g., \code
     GeographicLib::SphericalEngine::RootTable(2190);
     \endcode
     * suffices to accommodate extant magnetic and gravity models.
     **********************************************************************/
    static void RootTable(int N);

    /**
     * Clear the static table of square roots and release the memory.  Call
     * this only when you are sure you no longer will be using SphericalEngine.
     * Your program will crash if you call SphericalEngine after calling this
     * routine.
     *
     * \warning It's safest not to call this routine at all.  (The space used
     * by the table is modest.)
     **********************************************************************/
    static void ClearRootTable() {
      std::vector<real> temp(0);
      sqrttable().swap(temp);
    }
  };

} // namespace GeographicLib

#if defined(_MSC_VER)
#  pragma warning (pop)
#endif

#endif  // GEOGRAPHICLIB_SPHERICALENGINE_HPP

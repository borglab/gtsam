/**
 * \file DST.hpp
 * \brief Header for GeographicLib::DST class
 *
 * Copyright (c) Charles Karney (2022-2024) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_DST_HPP)
#define GEOGRAPHICLIB_DST_HPP 1

#include <GeographicLib/Constants.hpp>

#include <functional>
#include <memory>

/// \cond SKIP
template<typename scalar_t>
class kissfft;
/// \endcond

namespace GeographicLib {

  /**
   * \brief Discrete sine transforms
   *
   * This decomposes periodic functions \f$ f(\sigma) \f$ (period \f$ 2\pi \f$)
   * which are odd about \f$ \sigma = 0 \f$ and even about \f$ \sigma = \frac12
   * \pi \f$ into a Fourier series
   * \f[
   *   f(\sigma) = \sum_{l=0}^\infty F_l \sin\bigl((2l+1)\sigma\bigr).
   * \f]
   *
   * The first \f$ N \f$ components of \f$ F_l \f$, for \f$0 \le l < N\f$ may
   * be approximated by
   * \f[
   *   F_l = \frac2N \sum_{j=1}^{N}
   *   p_j f(\sigma_j)  \sin\bigl((2l+1)\sigma_j\bigr),
   * \f]
   * where \f$ \sigma_j = j\pi/(2N) \f$ and \f$ p_j = \frac12 \f$ for \f$ j = N
   * \f$ and \f$ 1 \f$ otherwise.  \f$ F_l \f$ is a discrete sine transform of
   * type DST-III and may be conveniently computed using the fast Fourier
   * transform, FFT; this is implemented with the DST::transform method.
   *
   * Having computed \f$ F_l \f$ based on \f$ N \f$ evaluations of \f$
   * f(\sigma) \f$ at \f$ \sigma_j = j\pi/(2N) \f$, it is possible to
   * refine these transform values and add another \f$ N \f$ coefficients by
   * evaluating \f$ f(\sigma) \f$ at \f$ (j-\frac12)\pi/(2N) \f$; this is
   * implemented with the DST::refine method.
   *
   * Here we compute FFTs using the kissfft package
   * https://github.com/mborgerding/kissfft by Mark Borgerding.
   *
   * Example of use:
   * \include example-DST.cpp
   *
   * \note The FFTW package https://www.fftw.org/ can also be used.  However
   * this is a more complicated dependency, its CMake support is broken, and it
   * doesn't work with mpreals (GEOGRAPHICLIB_PRECISION = 5).
   **********************************************************************/

  class DST {
  private:
    typedef Math::real real;
    int _nN;
    typedef kissfft<real> fft_t;
    std::shared_ptr<fft_t> _fft;
    // Implement DST-III (centerp = false) or DST-IV (centerp = true)
    void fft_transform(real data[], real F[], bool centerp) const;
    // Add another N terms to F
    void fft_transform2(real data[], real F[]) const;
  public:
    /**
     * Constructor specifying the number of points to use.
     *
     * @param[in] N the number of points to use.
     **********************************************************************/
    GEOGRAPHICLIB_EXPORT DST(int N = 0);

    /**
     * Reset the given number of points.
     *
     * @param[in] N the number of points to use.
     **********************************************************************/
    void GEOGRAPHICLIB_EXPORT reset(int N);

    /**
     * Return the number of points.
     *
     * @return the number of points to use.
     **********************************************************************/
    int N() const { return _nN; }

    /**
     * Determine first \e N terms in the Fourier series
     *
     * @param[in] f the function used for evaluation.
     * @param[out] F the first \e N coefficients of the Fourier series.
     *
     * The evaluates \f$ f(\sigma) \f$ at \f$ \sigma = (j + 1) \pi / (2 N) \f$
     * for integer \f$ j \in [0, N) \f$.  \e F should be an array of length at
     * least \e N.
     **********************************************************************/
    void GEOGRAPHICLIB_EXPORT transform(std::function<real(real)> f, real F[])
      const;

    /**
     * Refine the Fourier series by doubling the number of points sampled
     *
     * @param[in] f the function used for evaluation.
     * @param[inout] F on input the first \e N coefficents of the Fourier
     *   series; on output the refined transform based on 2\e N points, i.e.,
     *   the first 2\e N coefficents.
     *
     * The evaluates \f$ f(\sigma) \f$ at additional points \f$ \sigma = (j +
     * \frac12) \pi / (2 N) \f$ for integer \f$ j \in [0, N) \f$, computes the
     * DST-IV transform of these, and combines this with the input \e F to
     * compute the 2\e N term DST-III discrete sine transform.  This is
     * equivalent to calling transform with twice the value of \e N but is more
     * efficient, given that the \e N term coefficients are already known.  See
     * the example code above.
     **********************************************************************/
    void GEOGRAPHICLIB_EXPORT refine(std::function<real(real)> f, real F[])
      const;

    /**
     * Evaluate the Fourier sum given the sine and cosine of the angle
     *
     * @param[in] sinx sin&sigma;.
     * @param[in] cosx cos&sigma;.
     * @param[in] F the array of Fourier coefficients.
     * @param[in] N the number of Fourier coefficients.
     * @return the value of the Fourier sum.
     **********************************************************************/
    static real GEOGRAPHICLIB_EXPORT eval(real sinx, real cosx,
                                          const real F[], int N);

    /**
     * Evaluate the integral of Fourier sum given the sine and cosine of the
     * angle
     *
     * @param[in] sinx sin&sigma;.
     * @param[in] cosx cos&sigma;.
     * @param[in] F the array of Fourier coefficients.
     * @param[in] N the number of Fourier coefficients.
     * @return the value of the integral.
     *
     * The constant of integration is chosen so that the integral is zero at
     * \f$ \sigma = \frac12\pi \f$.
     **********************************************************************/
    static real GEOGRAPHICLIB_EXPORT integral(real sinx, real cosx,
                                              const real F[], int N);

    /**
     * Evaluate the definite integral of Fourier sum given the sines and
     * cosines of the angles at the endpoints.
     *
     * @param[in] sinx sin&sigma;<sub>1</sub>.
     * @param[in] cosx cos&sigma;<sub>1</sub>.
     * @param[in] siny sin&sigma;<sub>2</sub>.
     * @param[in] cosy cos&sigma;<sub>2</sub>.
     * @param[in] F the array of Fourier coefficients.
     * @param[in] N the number of Fourier coefficients.
     * @return the value of the integral.
     *
     * The integral is evaluated between limits &sigma;<sub>1</sub> and
     * &sigma;<sub>2</sub>.
     **********************************************************************/
    static real GEOGRAPHICLIB_EXPORT integral(real sinx, real cosx,
                                              real siny, real cosy,
                                              const real F[], int N);
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_DST_HPP

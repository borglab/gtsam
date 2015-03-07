/**
 * \file EllipticFunction.hpp
 * \brief Header for GeographicLib::EllipticFunction class
 *
 * Copyright (c) Charles Karney (2008-2012) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_ELLIPTICFUNCTION_HPP)
#define GEOGRAPHICLIB_ELLIPTICFUNCTION_HPP 1

#include <GeographicLib/Constants.hpp>

namespace GeographicLib {

  /**
   * \brief Elliptic integrals and functions
   *
   * This provides the elliptic functions and integrals needed for Ellipsoid,
   * GeodesicExact, and TransverseMercatorExact.  Two categories of function
   * are provided:
   * - \e static functions to compute symmetric elliptic integrals
   *   (http://dlmf.nist.gov/19.16.i)
   * - \e member functions to compute Legrendre's elliptic
   *   integrals (http://dlmf.nist.gov/19.2.ii) and the
   *   Jacobi elliptic functions (http://dlmf.nist.gov/22.2).
   * .
   * In the latter case, an object is constructed giving the modulus \e k (and
   * optionally the parameter &alpha;<sup>2</sup>).  The modulus is always
   * passed as its square <i>k</i><sup>2</sup> which allows \e k to be pure
   * imaginary (<i>k</i><sup>2</sup> &lt; 0).  (Confusingly, Abramowitz and
   * Stegun call \e m = <i>k</i><sup>2</sup> the "parameter" and \e n =
   * &alpha;<sup>2</sup> the "characteristic".)
   *
   * In geodesic applications, it is convenient to separate the incomplete
   * integrals into secular and periodic components, e.g.,
   * \f[
   *   E(\phi, k) = (2 E(\phi) / \pi) [ \phi + \delta E(\phi, k) ]
   * \f]
   * where &delta;\e E(&phi;, \e k) is an odd periodic function with period
   * &pi;.
   *
   * The computation of the elliptic integrals uses the algorithms given in
   * - B. C. Carlson,
   *   <a href="http://dx.doi.org/10.1007/BF02198293"> Computation of real or
   *   complex elliptic integrals</a>, Numerical Algorithms 10, 13--26 (1995)
   * .
   * with the additional optimizations given in http://dlmf.nist.gov/19.36.i.
   * The computation of the Jacobi elliptic functions uses the algorithm given
   * in
   * - R. Bulirsch,
   *   <a href="http://dx.doi.org/10.1007/BF01397975"> Numerical Calculation of
   *   Elliptic Integrals and Elliptic Functions</a>, Numericshe Mathematik 7,
   *   78--90 (1965).
   * .
   * The notation follows http://dlmf.nist.gov/19 and http://dlmf.nist.gov/22
   *
   * Example of use:
   * \include example-EllipticFunction.cpp
   **********************************************************************/
  class GEOGRAPHICLIB_EXPORT EllipticFunction {
  private:
    typedef Math::real real;
    static const real tol_;
    static const real tolRF_;
    static const real tolRD_;
    static const real tolRG0_;
    static const real tolJAC_;
    enum { num_ = 13 }; // Max depth required for sncndn.  Probably 5 is enough.
    real _k2, _kp2, _alpha2, _alphap2, _eps;
    mutable bool _init;
    mutable real _Kc, _Ec, _Dc, _Pic, _Gc, _Hc;
    bool Init() const throw();
  public:
    /** \name Constructor
     **********************************************************************/
    ///@{
    /**
     * Constructor specifying the modulus and parameter.
     *
     * @param[in] k2 the square of the modulus <i>k</i><sup>2</sup>.
     *   <i>k</i><sup>2</sup> must lie in (-&infin;, 1).  (No checking is
     *   done.)
     * @param[in] alpha2 the parameter &alpha;<sup>2</sup>.
     *   &alpha;<sup>2</sup> must lie in (-&infin;, 1).  (No checking is done.)
     *
     * If only elliptic integrals of the first and second kinds are needed,
     * then set &alpha;<sup>2</sup> = 0 (the default value); in this case, we
     * have &Pi;(&phi;, 0, \e k) = \e F(&phi;, \e k), \e G(&phi;, 0, \e k) = \e
     * E(&phi;, \e k), and \e H(&phi;, 0, \e k) = \e F(&phi;, \e k) - \e
     * D(&phi;, \e k).
     **********************************************************************/
    EllipticFunction(real k2 = 0, real alpha2 = 0) throw();

    /**
     * Constructor specifying the modulus and parameter and their complements.
     *
     * @param[in] k2 the square of the modulus <i>k</i><sup>2</sup>.
     *   <i>k</i><sup>2</sup> must lie in (-&infin;, 1).  (No checking is
     *   done.)
     * @param[in] alpha2 the parameter &alpha;<sup>2</sup>.
     *   &alpha;<sup>2</sup> must lie in (-&infin;, 1).  (No checking is done.)
     * @param[in] kp2 the complementary modulus squared <i>k'</i><sup>2</sup> =
     *   1 &minus; <i>k</i><sup>2</sup>.
     * @param[in] alphap2 the complementary parameter &alpha;'<sup>2</sup> = 1
     *   &minus; &alpha;<sup>2</sup>.
     *
     * The arguments must satisfy \e k2 + \e kp2 = 1 and \e alpha2 + \e alphap2
     * = 1.  (No checking is done that these conditions are met.)  This
     * constructor is provided to enable accuracy to be maintained, e.g., when
     * \e k is very close to unity.
     **********************************************************************/
    EllipticFunction(real k2, real alpha2, real kp2, real alphap2) throw();

    /**
     * Reset the modulus and parameter.
     *
     * @param[in] k2 the new value of square of the modulus
     *   <i>k</i><sup>2</sup> which must lie in (-&infin;, 1).  (No checking is
     *   done.)
     * @param[in] alpha2 the new value of parameter &alpha;<sup>2</sup>.
     *   &alpha;<sup>2</sup> must lie in (-&infin;, 1).  (No checking is done.)
     **********************************************************************/
    void Reset(real k2 = 0, real alpha2 = 0) throw()
    { Reset(k2, alpha2, 1 - k2, 1 - alpha2); }

    /**
     * Reset the modulus and parameter supplying also their complements.
     *
     * @param[in] k2 the square of the modulus <i>k</i><sup>2</sup>.
     *   <i>k</i><sup>2</sup> must lie in (-&infin;, 1).  (No checking is
     *   done.)
     * @param[in] alpha2 the parameter &alpha;<sup>2</sup>.
     *   &alpha;<sup>2</sup> must lie in (-&infin;, 1).  (No checking is done.)
     * @param[in] kp2 the complementary modulus squared <i>k'</i><sup>2</sup> =
     *   1 &minus; <i>k</i><sup>2</sup>.
     * @param[in] alphap2 the complementary parameter &alpha;'<sup>2</sup> = 1
     *   &minus; &alpha;<sup>2</sup>.
     *
     * The arguments must satisfy \e k2 + \e kp2 = 1 and \e alpha2 + \e alphap2
     * = 1.  (No checking is done that these conditions are met.)  This
     * constructor is provided to enable accuracy to be maintained, e.g., when
     * is very small.
     **********************************************************************/
    void Reset(real k2, real alpha2, real kp2, real alphap2) throw();

    ///@}

    /** \name Inspector functions.
     **********************************************************************/
    ///@{
    /**
     * @return the square of the modulus <i>k</i><sup>2</sup>.
     **********************************************************************/
    Math::real k2() const throw() { return _k2; }

    /**
     * @return the square of the complementary modulus <i>k'</i><sup>2</sup> =
     *   1 &minus; <i>k</i><sup>2</sup>.
     **********************************************************************/
    Math::real kp2() const throw() { return _kp2; }

    /**
     * @return the parameter &alpha;<sup>2</sup>.
     **********************************************************************/
    Math::real alpha2() const throw() { return _alpha2; }

    /**
     * @return the complementary parameter &alpha;'<sup>2</sup> = 1 &minus;
     *   &alpha;<sup>2</sup>.
     **********************************************************************/
    Math::real alphap2() const throw() { return _alphap2; }
    ///@}

    /// \cond SKIP
    /**
     * @return the square of the modulus <i>k</i><sup>2</sup>.
     *
     * <b>DEPRECATED</b>, use k2() instead.
     **********************************************************************/
    Math::real m() const throw() { return _k2; }
    /**
     * @return the square of the complementary modulus <i>k'</i><sup>2</sup> =
     *   1 &minus; <i>k</i><sup>2</sup>.
     *
     * <b>DEPRECATED</b>, use kp2() instead.
     **********************************************************************/
    Math::real m1() const throw() { return _kp2; }
    /// \endcond

    /** \name Complete elliptic integrals.
     **********************************************************************/
    ///@{
    /**
     * The complete integral of the first kind.
     *
     * @return \e K(\e k).
     *
     * \e K(\e k) is defined in http://dlmf.nist.gov/19.2.E4
     * \f[
     *   K(k) = \int_0^{\pi/2} \frac1{\sqrt{1-k^2\sin^2\phi}}\,d\phi.
     * \f]
     **********************************************************************/
    Math::real K() const throw() { _init || Init(); return _Kc; }

    /**
     * The complete integral of the second kind.
     *
     * @return \e E(\e k)
     *
     * \e E(\e k) is defined in http://dlmf.nist.gov/19.2.E5
     * \f[
     *   E(k) = \int_0^{\pi/2} \sqrt{1-k^2\sin^2\phi}\,d\phi.
     * \f]
     **********************************************************************/
    Math::real E() const throw() { _init || Init(); return _Ec; }

    /**
     * Jahnke's complete integral.
     *
     * @return \e D(\e k).
     *
     * \e D(\e k) is defined in http://dlmf.nist.gov/19.2.E6
     * \f[
     *   D(k) = \int_0^{\pi/2} \frac{\sin^2\phi}{\sqrt{1-k^2\sin^2\phi}}\,d\phi.
     * \f]
     **********************************************************************/
    Math::real D() const throw() { _init || Init(); return _Dc; }

    /**
     * The difference between the complete integrals of the first and second
     * kinds.
     *
     * @return \e K(\e k) &minus; \e E(\e k).
     **********************************************************************/
    Math::real KE() const throw() { _init || Init(); return _k2 * _Dc; }

    /**
     * The complete integral of the third kind.
     *
     * @return &Pi;(&alpha;<sup>2</sup>, \e k)
     *
     * &Pi;(&alpha;<sup>2</sup>, \e k) is defined in
     * http://dlmf.nist.gov/19.2.E7
     * \f[
     *   \Pi(\alpha^2, k) = \int_0^{\pi/2}
     *     \frac1{\sqrt{1-k^2\sin^2\phi}(1 - \alpha^2\sin^2\phi_)}\,d\phi.
     * \f]
     **********************************************************************/
    Math::real Pi() const throw() { _init || Init(); return _Pic; }

    /**
     * Legendre's complete geodesic longitude integral.
     *
     * @return \e G(&alpha;<sup>2</sup>, \e k)
     *
     * \e G(&alpha;<sup>2</sup>, \e k) is given by
     * \f[
     *   G(\alpha^2, k) = \int_0^{\pi/2}
     *     \frac{\sqrt{1-k^2\sin^2\phi}}{1 - \alpha^2\sin^2\phi}\,d\phi.
     * \f]
     **********************************************************************/
    Math::real G() const throw() { _init || Init(); return _Gc; }

    /**
     * Cayley's complete geodesic longitude difference integral.
     *
     * @return \e H(&alpha;<sup>2</sup>, \e k)
     *
     * \e H(&alpha;<sup>2</sup>, \e k) is given by
     * \f[
     *   H(\alpha^2, k) = \int_0^{\pi/2}
     *     \frac{\cos^2\phi}{(1-\alpha^2\sin^2\phi)\sqrt{1-k^2\sin^2\phi}}
     *     \,d\phi.
     * \f]
     **********************************************************************/
    Math::real H() const throw() { _init || Init(); return _Hc; }
    ///@}

    /** \name Incomplete elliptic integrals.
     **********************************************************************/
    ///@{
    /**
     * The incomplete integral of the first kind.
     *
     * @param[in] phi
     * @return \e F(&phi;, \e k).
     *
     * \e F(&phi;, \e k) is defined in http://dlmf.nist.gov/19.2.E4
     * \f[
     *   F(\phi, k) = \int_0^\phi \frac1{\sqrt{1-k^2\sin^2\theta}}\,d\theta.
     * \f]
     **********************************************************************/
    Math::real F(real phi) const throw();

    /**
     * The incomplete integral of the second kind.
     *
     * @param[in] phi
     * @return \e E(&phi;, \e k).
     *
     * \e E(&phi;, \e k) is defined in http://dlmf.nist.gov/19.2.E5
     * \f[
     *   E(\phi, k) = \int_0^\phi \sqrt{1-k^2\sin^2\theta}\,d\theta.
     * \f]
     **********************************************************************/
    Math::real E(real phi) const throw();

    /**
     * The incomplete integral of the second kind with the argument given in
     * degrees.
     *
     * @param[in] ang in <i>degrees</i>.
     * @return \e E(&pi; <i>ang</i>/180, \e k).
     **********************************************************************/
    Math::real Ed(real ang) const throw();

    /**
     * The inverse of the incomplete integral of the second kind.
     *
     * @param[in] x
     * @return &phi; = <i>E</i><sup>&minus;1</sup>(\e x, \e k); i.e., the
     *   solution of such that \e E(&phi;, \e k) = \e x.
     **********************************************************************/
    Math::real Einv(real x) const throw();

    /**
     * The incomplete integral of the third kind.
     *
     * @param[in] phi
     * @return &Pi;(&phi;, &alpha;<sup>2</sup>, \e k).
     *
     * &Pi;(&phi;, &alpha;<sup>2</sup>, \e k) is defined in
     * http://dlmf.nist.gov/19.2.E7
     * \f[
     *   \Pi(\phi, \alpha^2, k) = \int_0^\phi
     *     \frac1{\sqrt{1-k^2\sin^2\theta}(1 - \alpha^2\sin^2\theta_)}\,d\theta.
     * \f]
     **********************************************************************/
    Math::real Pi(real phi) const throw();

    /**
     * Jahnke's incomplete elliptic integral.
     *
     * @param[in] phi
     * @return \e D(&phi;, \e k).
     *
     * \e D(&phi;, \e k) is defined in http://dlmf.nist.gov/19.2.E4
     * \f[
     *   D(\phi, k) = \int_0^\phi
     *    \frac{\sin^2\theta}{\sqrt{1-k^2\sin^2\theta}}\,d\theta.
     * \f]
     **********************************************************************/
    Math::real D(real phi) const throw();

    /**
     * Legendre's geodesic longitude integral.
     *
     * @param[in] phi
     * @return \e G(&phi;, &alpha;<sup>2</sup>, \e k).
     *
     * \e G(&phi;, &alpha;<sup>2</sup>, \e k) is defined by
     * \f[
     *   \begin{aligned}
     *   G(\phi, \alpha^2, k) &=
     *   \frac{k^2}{\alpha^2} F(\phi, k) +
     *      \biggl(1 - \frac{k^2}{\alpha^2}\biggr) \Pi(\phi, \alpha^2, k) \\
     *    &= \int_0^\phi
     *     \frac{\sqrt{1-k^2\sin^2\theta}}{1 - \alpha^2\sin^2\theta}\,d\theta.
     *   \end{aligned}
     * \f]
     *
     * Legendre expresses the longitude of a point on the geodesic in terms of
     * this combination of elliptic integrals in Exercices de Calcul
     * Int&eacute;gral, Vol. 1 (1811), p. 181,
     * http://books.google.com/books?id=riIOAAAAQAAJ&pg=PA181.
     *
     * See \ref geodellip for the expression for the longitude in terms of this
     * function.
     **********************************************************************/
    Math::real G(real phi) const throw();

    /**
     * Cayley's geodesic longitude difference integral.
     *
     * @param[in] phi
     * @return \e H(&phi;, &alpha;<sup>2</sup>, \e k).
     *
     * \e H(&phi;, &alpha;<sup>2</sup>, \e k) is defined by
     * \f[
     *   \begin{aligned}
     *   H(\phi, \alpha^2, k) &=
     *   \frac1{\alpha^2} F(\phi, k) +
     *        \biggl(1 - \frac1{\alpha^2}\biggr) \Pi(\phi, \alpha^2, k) \\
     *   &= \int_0^\phi
     *     \frac{\cos^2\theta}{(1-\alpha^2\sin^2\theta)\sqrt{1-k^2\sin^2\theta}}
     *     \,d\theta.
     *   \end{aligned}
     * \f]
     *
     * Cayley expresses the longitude difference of a point on the geodesic in
     * terms of this combination of elliptic integrals in Phil. Mag. <b>40</b>
     * (1870), p. 333, http://books.google.com/books?id=Zk0wAAAAIAAJ&pg=PA333.
     *
     * See \ref geodellip for the expression for the longitude in terms of this
     * function.
     **********************************************************************/
    Math::real H(real phi) const throw();
    ///@}

    /** \name Incomplete integrals in terms of Jacobi elliptic functions.
     **********************************************************************/
    /**
     * The incomplete integral of the first kind in terms of Jacobi elliptic
     * functions.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return \e F(&phi;, \e k) as though &phi; &isin; (&minus;&pi;, &pi;].
     **********************************************************************/
    Math::real F(real sn, real cn, real dn) const throw();

    /**
     * The incomplete integral of the second kind in terms of Jacobi elliptic
     * functions.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return \e E(&phi;, \e k) as though &phi; &isin; (&minus;&pi;, &pi;].
     **********************************************************************/
    Math::real E(real sn, real cn, real dn) const throw();

    /**
     * The incomplete integral of the third kind in terms of Jacobi elliptic
     * functions.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return &Pi;(&phi;, &alpha;<sup>2</sup>, \e k) as though &phi; &isin;
     *   (&minus;&pi;, &pi;].
     **********************************************************************/
    Math::real Pi(real sn, real cn, real dn) const throw();

    /**
     * Jahnke's incomplete elliptic integral in terms of Jacobi elliptic
     * functions.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return \e D(&phi;, \e k) as though &phi; &isin; (&minus;&pi;, &pi;].
     **********************************************************************/
    Math::real D(real sn, real cn, real dn) const throw();

    /**
     * Legendre's geodesic longitude integral in terms of Jacobi elliptic
     * functions.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return \e G(&phi;, &alpha;<sup>2</sup>, \e k) as though &phi; &isin;
     *   (&minus;&pi;, &pi;].
     **********************************************************************/
    Math::real G(real sn, real cn, real dn) const throw();

    /**
     * Cayley's geodesic longitude difference integral in terms of Jacobi
     * elliptic functions.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return \e H(&phi;, &alpha;<sup>2</sup>, \e k) as though &phi; &isin;
     *   (&minus;&pi;, &pi;].
     **********************************************************************/
    Math::real H(real sn, real cn, real dn) const throw();
    ///@}

    /** \name Periodic versions of incomplete elliptic integrals.
     **********************************************************************/
    ///@{
    /**
     * The periodic incomplete integral of the first kind.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return the periodic function &pi; \e F(&phi;, \e k) / (2 \e K(\e k)) -
     *   &phi;
     **********************************************************************/
    Math::real deltaF(real sn, real cn, real dn) const throw();

    /**
     * The periodic incomplete integral of the second kind.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return the periodic function &pi; \e E(&phi;, \e k) / (2 \e E(\e k)) -
     *   &phi;
     **********************************************************************/
    Math::real deltaE(real sn, real cn, real dn) const throw();

    /**
     * The periodic inverse of the incomplete integral of the second kind.
     *
     * @param[in] stau = sin&tau;
     * @param[in] ctau = sin&tau;
     * @return the periodic function <i>E</i><sup>&minus;1</sup>(&tau; (2 \e
     *   E(\e k)/&pi;), \e k) - &tau;
     **********************************************************************/
    Math::real deltaEinv(real stau, real ctau) const throw();

    /**
     * The periodic incomplete integral of the third kind.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return the periodic function &pi; &Pi;(&phi;, \e k) / (2 &Pi;(\e k)) -
     *   &phi;
     **********************************************************************/
    Math::real deltaPi(real sn, real cn, real dn) const throw();

    /**
     * The periodic Jahnke's incomplete elliptic integral.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return the periodic function &pi; \e D(&phi;, \e k) / (2 \e D(\e k)) -
     *   &phi;
     **********************************************************************/
    Math::real deltaD(real sn, real cn, real dn) const throw();

    /**
     * Legendre's periodic geodesic longitude integral.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return the periodic function &pi; \e G(&phi;, \e k) / (2 \e G(\e k)) -
     *   &phi;
     **********************************************************************/
    Math::real deltaG(real sn, real cn, real dn) const throw();

    /**
     * Cayley's periodic geodesic longitude difference integral.
     *
     * @param[in] sn = sin&phi;
     * @param[in] cn = cos&phi;
     * @param[in] dn = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     * @return the periodic function &pi; \e H(&phi;, \e k) / (2 \e H(\e k)) -
     *   &phi;
     **********************************************************************/
    Math::real deltaH(real sn, real cn, real dn) const throw();
    ///@}

    /** \name Elliptic functions.
     **********************************************************************/
    ///@{
    /**
     * The Jacobi elliptic functions.
     *
     * @param[in] x the argument.
     * @param[out] sn sn(\e x, \e k).
     * @param[out] cn cn(\e x, \e k).
     * @param[out] dn dn(\e x, \e k).
     **********************************************************************/
    void sncndn(real x, real& sn, real& cn, real& dn) const throw();

    /**
     * The &Delta; amplitude function.
     *
     * @param[in] sn sin&phi;
     * @param[in] cn cos&phi;
     * @return &Delta; = sqrt(1 &minus; <i>k</i><sup>2</sup>
     *   sin<sup>2</sup>&phi;)
     **********************************************************************/
    Math::real Delta(real sn, real cn) const throw()
    { return std::sqrt(_k2 < 0 ? 1 - _k2 * sn*sn : _kp2 + _k2 * cn*cn); }
    ///@}

    /** \name Symmetric elliptic integrals.
     **********************************************************************/
    ///@{
    /**
     * Symmetric integral of the first kind <i>R<sub>F</sub></i>.
     *
     * @param[in] x
     * @param[in] y
     * @param[in] z
     * @return <i>R<sub>F</sub></i>(\e x, \e y, \e z)
     *
     * <i>R<sub>F</sub></i> is defined in http://dlmf.nist.gov/19.16.E1
     * \f[ R_F(x, y, z) = \frac12
     *       \int_0^\infty\frac1{\sqrt{(t + x) (t + y) (t + z)}}\, dt \f]
     * If one of the arguments is zero, it is more efficient to call the
     * two-argument version of this function with the non-zero arguments.
     **********************************************************************/
    static real RF(real x, real y, real z) throw();

    /**
     * Complete symmetric integral of the first kind, <i>R<sub>F</sub></i> with
     * one argument zero.
     *
     * @param[in] x
     * @param[in] y
     * @return <i>R<sub>F</sub></i>(\e x, \e y, 0)
     **********************************************************************/
    static real RF(real x, real y) throw();

    /**
     * Degenerate symmetric integral of the first kind <i>R<sub>C</sub></i>.
     *
     * @param[in] x
     * @param[in] y
     * @return <i>R<sub>C</sub></i>(\e x, \e y) = <i>R<sub>F</sub></i>(\e x, \e
     *   y, \e y)
     *
     * <i>R<sub>C</sub></i> is defined in http://dlmf.nist.gov/19.2.E17
     * \f[ R_C(x, y) = \frac12
     *       \int_0^\infty\frac1{\sqrt{t + x}(t + y)}\,dt \f]
     **********************************************************************/
    static real RC(real x, real y) throw();

    /**
     * Symmetric integral of the second kind <i>R<sub>G</sub></i>.
     *
     * @param[in] x
     * @param[in] y
     * @param[in] z
     * @return <i>R<sub>G</sub></i>(\e x, \e y, \e z)
     *
     * <i>R<sub>G</sub></i> is defined in Carlson, eq 1.5
     * \f[ R_G(x, y, z) = \frac14
     *       \int_0^\infty[(t + x) (t + y) (t + z)]^{-1/2}
     *        \biggl(
     *             \frac x{t + x} + \frac y{t + y} + \frac z{t + z}
     *        \biggr)t\,dt \f]
     * See also http://dlmf.nist.gov/19.16.E3.
     * If one of the arguments is zero, it is more efficient to call the
     * two-argument version of this function with the non-zero arguments.
     **********************************************************************/
    static real RG(real x, real y, real z) throw();

    /**
     * Complete symmetric integral of the second kind, <i>R<sub>G</sub></i>
     * with one argument zero.
     *
     * @param[in] x
     * @param[in] y
     * @return <i>R<sub>G</sub></i>(\e x, \e y, 0)
     **********************************************************************/
    static real RG(real x, real y) throw();

    /**
     * Symmetric integral of the third kind <i>R<sub>J</sub></i>.
     *
     * @param[in] x
     * @param[in] y
     * @param[in] z
     * @param[in] p
     * @return <i>R<sub>J</sub></i>(\e x, \e y, \e z, \e p)
     *
     * <i>R<sub>J</sub></i> is defined in http://dlmf.nist.gov/19.16.E2
     * \f[ R_J(x, y, z, p) = \frac32
     *       \int_0^\infty[(t + x) (t + y) (t + z)]^{-1/2} (t + p)^{-1}\, dt \f]
     **********************************************************************/
    static real RJ(real x, real y, real z, real p) throw();

    /**
     * Degenerate symmetric integral of the third kind <i>R<sub>D</sub></i>.
     *
     * @param[in] x
     * @param[in] y
     * @param[in] z
     * @return <i>R<sub>D</sub></i>(\e x, \e y, \e z) = <i>R<sub>J</sub></i>(\e
     *   x, \e y, \e z, \e z)
     *
     * <i>R<sub>D</sub></i> is defined in http://dlmf.nist.gov/19.16.E5
     * \f[ R_D(x, y, z) = \frac32
     *       \int_0^\infty[(t + x) (t + y)]^{-1/2} (t + z)^{-3/2}\, dt \f]
     **********************************************************************/
    static real RD(real x, real y, real z) throw();
    ///@}

  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_ELLIPTICFUNCTION_HPP

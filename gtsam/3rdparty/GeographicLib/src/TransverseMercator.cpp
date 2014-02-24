/**
 * \file TransverseMercator.cpp
 * \brief Implementation for GeographicLib::TransverseMercator class
 *
 * Copyright (c) Charles Karney (2008-2012) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 *
 * This implementation follows closely
 * <a href="http://www.jhs-suositukset.fi/suomi/jhs154"> JHS 154, ETRS89 -
 * j&auml;rjestelm&auml;&auml;n liittyv&auml;t karttaprojektiot,
 * tasokoordinaatistot ja karttalehtijako</a> (Map projections, plane
 * coordinates, and map sheet index for ETRS89), published by JUHTA, Finnish
 * Geodetic Institute, and the National Land Survey of Finland (2006).
 *
 * The relevant section is available as the 2008 PDF file
 * http://docs.jhs-suositukset.fi/jhs-suositukset/JHS154/JHS154_liite1.pdf
 *
 * This is a straight transcription of the formulas in this paper with the
 * following exceptions:
 *  - use of 6th order series instead of 4th order series.  This reduces the
 *    error to about 5nm for the UTM range of coordinates (instead of 200nm),
 *    with a speed penalty of only 1%;
 *  - use Newton's method instead of plain iteration to solve for latitude in
 *    terms of isometric latitude in the Reverse method;
 *  - use of Horner's representation for evaluating polynomials and Clenshaw's
 *    method for summing trigonometric series;
 *  - several modifications of the formulas to improve the numerical accuracy;
 *  - evaluating the convergence and scale using the expression for the
 *    projection or its inverse.
 *
 * If the preprocessor variable GEOGRAPHICLIB_TRANSVERSEMERCATOR_ORDER is set
 * to an integer between 4 and 8, then this specifies the order of the series
 * used for the forward and reverse transformations.  The default value is 6.
 * (The series accurate to 12th order is given in \ref tmseries.)
 *
 * Other equivalent implementations are given in
 *  - http://www.ign.fr/DISPLAY/000/526/702/5267021/NTG_76.pdf
 *  - http://www.lantmateriet.se/upload/filer/kartor/geodesi_gps_och_detaljmatning/geodesi/Formelsamling/Gauss_Conformal_Projection.pdf
 **********************************************************************/

#include <GeographicLib/TransverseMercator.hpp>

namespace GeographicLib {

  using namespace std;

  const Math::real TransverseMercator::tol_ =
    real(0.1)*sqrt(numeric_limits<real>::epsilon());
  // Overflow value s.t. atan(overflow_) = pi/2
  const Math::real TransverseMercator::overflow_ =
    1 / Math::sq(numeric_limits<real>::epsilon());

  TransverseMercator::TransverseMercator(real a, real f, real k0)
    : _a(a)
    , _f(f <= 1 ? f : 1/f)
    , _k0(k0)
    , _e2(_f * (2 - _f))
    , _e(sqrt(abs(_e2)))
    , _e2m(1 - _e2)
      // _c = sqrt( pow(1 + _e, 1 + _e) * pow(1 - _e, 1 - _e) ) )
      // See, for example, Lee (1976), p 100.
    , _c( sqrt(_e2m) * exp(eatanhe(real(1))) )
    , _n(_f / (2 - _f))
  {
    if (!(Math::isfinite(_a) && _a > 0))
      throw GeographicErr("Major radius is not positive");
    if (!(Math::isfinite(_f) && _f < 1))
      throw GeographicErr("Minor radius is not positive");
    if (!(Math::isfinite(_k0) && _k0 > 0))
      throw GeographicErr("Scale is not positive");
    // If coefficents might overflow_ an int, convert them to double (and they
    // are all exactly representable as doubles).
    real nx = Math::sq(_n);
    switch (maxpow_) {
    case 4:
      _b1 = 1/(1+_n)*(nx*(nx+16)+64)/64;
      _alp[1] = _n*(_n*(_n*(164*_n+225)-480)+360)/720;
      _bet[1] = _n*(_n*((555-4*_n)*_n-960)+720)/1440;
      _alp[2] = nx*(_n*(557*_n-864)+390)/1440;
      _bet[2] = nx*((96-437*_n)*_n+30)/1440;
      nx *= _n;
      _alp[3] = (427-1236*_n)*nx/1680;
      _bet[3] = (119-148*_n)*nx/3360;
      nx *= _n;
      _alp[4] = 49561*nx/161280;
      _bet[4] = 4397*nx/161280;
      break;
    case 5:
      _b1 = 1/(1+_n)*(nx*(nx+16)+64)/64;
      _alp[1] = _n*(_n*(_n*((328-635*_n)*_n+450)-960)+720)/1440;
      _bet[1] = _n*(_n*(_n*((-3645*_n-64)*_n+8880)-15360)+11520)/23040;
      _alp[2] = nx*(_n*(_n*(4496*_n+3899)-6048)+2730)/10080;
      _bet[2] = nx*(_n*(_n*(4416*_n-3059)+672)+210)/10080;
      nx *= _n;
      _alp[3] = nx*(_n*(15061*_n-19776)+6832)/26880;
      _bet[3] = nx*((-627*_n-592)*_n+476)/13440;
      nx *= _n;
      _alp[4] = (49561-171840*_n)*nx/161280;
      _bet[4] = (4397-3520*_n)*nx/161280;
      nx *= _n;
      _alp[5] = 34729*nx/80640;
      _bet[5] = 4583*nx/161280;
      break;
    case 6:
      _b1 = 1/(1+_n)*(nx*(nx*(nx+4)+64)+256)/256;
      _alp[1] = _n*(_n*(_n*(_n*(_n*(31564*_n-66675)+34440)+47250)-100800)+
                    75600)/151200;
      _bet[1] = _n*(_n*(_n*(_n*(_n*(384796*_n-382725)-6720)+932400)-1612800)+
                    1209600)/2419200;
      _alp[2] = nx*(_n*(_n*((863232-1983433*_n)*_n+748608)-1161216)+524160)/
        1935360;
      _bet[2] = nx*(_n*(_n*((1695744-1118711*_n)*_n-1174656)+258048)+80640)/
        3870720;
      nx *= _n;
      _alp[3] = nx*(_n*(_n*(670412*_n+406647)-533952)+184464)/725760;
      _bet[3] = nx*(_n*(_n*(22276*_n-16929)-15984)+12852)/362880;
      nx *= _n;
      _alp[4] = nx*(_n*(6601661*_n-7732800)+2230245)/7257600;
      _bet[4] = nx*((-830251*_n-158400)*_n+197865)/7257600;
      nx *= _n;
      _alp[5] = (3438171-13675556*_n)*nx/7983360;
      _bet[5] = (453717-435388*_n)*nx/15966720;
      nx *= _n;
      _alp[6] = 212378941*nx/319334400;
      _bet[6] = 20648693*nx/638668800;
      break;
    case 7:
      _b1 = 1/(1+_n)*(nx*(nx*(nx+4)+64)+256)/256;
      _alp[1] = _n*(_n*(_n*(_n*(_n*(_n*(1804025*_n+2020096)-4267200)+2204160)+
                            3024000)-6451200)+4838400)/9676800;
      _bet[1] = _n*(_n*(_n*(_n*(_n*((6156736-5406467*_n)*_n-6123600)-107520)+
                            14918400)-25804800)+19353600)/38707200;
      _alp[2] = nx*(_n*(_n*(_n*(_n*(4626384*_n-9917165)+4316160)+3743040)-
                        5806080)+2620800)/9676800;
      _bet[2] = nx*(_n*(_n*(_n*(_n*(829456*_n-5593555)+8478720)-5873280)+
                        1290240)+403200)/19353600;
      nx *= _n;
      _alp[3] = nx*(_n*(_n*((26816480-67102379*_n)*_n+16265880)-21358080)+
                    7378560)/29030400;
      _bet[3] = nx*(_n*(_n*(_n*(9261899*_n+3564160)-2708640)-2557440)+
                    2056320)/58060800;
      nx *= _n;
      _alp[4] = nx*(_n*(_n*(155912000*_n+72618271)-85060800)+24532695)/
        79833600;
      _bet[4] = nx*(_n*(_n*(14928352*_n-9132761)-1742400)+2176515)/79833600;
      nx *= _n;
      _alp[5] = nx*(_n*(102508609*_n-109404448)+27505368)/63866880;
      _bet[5] = nx*((-8005831*_n-1741552)*_n+1814868)/63866880;
      nx *= _n;
      _alp[6] = (2760926233.0-12282192400.0*_n)*nx/4151347200.0;
      _bet[6] = (268433009-261810608*_n)*nx/8302694400.0;
      nx *= _n;
      _alp[7] = 1522256789.0*nx/1383782400.0;
      _bet[7] = 219941297*nx/5535129600.0;
      break;
    case 8:
      _b1 = 1/(1+_n)*(nx*(nx*(nx*(25*nx+64)+256)+4096)+16384)/16384;
      _alp[1] = _n*(_n*(_n*(_n*(_n*(_n*((37884525-75900428*_n)*_n+42422016)-
                                    89611200)+46287360)+63504000)-135475200)+
                    101606400)/203212800;
      _bet[1] = _n*(_n*(_n*(_n*(_n*(_n*(_n*(31777436*_n-37845269)+43097152)-
                                    42865200)-752640)+104428800)-180633600)+
                    135475200)/270950400;
      _alp[2] = nx*(_n*(_n*(_n*(_n*(_n*(148003883*_n+83274912)-178508970)+
                                77690880)+67374720)-104509440)+47174400)/
        174182400;
      _bet[2] = nx*(_n*(_n*(_n*(_n*(_n*(24749483*_n+14930208)-100683990)+
                                152616960)-105719040)+23224320)+7257600)/
        348364800;
      nx *= _n;
      _alp[3] = nx*(_n*(_n*(_n*(_n*(318729724*_n-738126169)+294981280)+
                            178924680)-234938880)+81164160)/319334400;
      _bet[3] = nx*(_n*(_n*(_n*((101880889-232468668*_n)*_n+39205760)-
                            29795040)-28131840)+22619520)/638668800;
      nx *= _n;
      _alp[4] = nx*(_n*(_n*((14967552000.0-40176129013.0*_n)*_n+6971354016.0)-
                        8165836800.0)+2355138720.0)/7664025600.0;
      _bet[4] = nx*(_n*(_n*(_n*(324154477*_n+1433121792.0)-876745056)-
                        167270400)+208945440)/7664025600.0;
      nx *= _n;
      _alp[5] = nx*(_n*(_n*(10421654396.0*_n+3997835751.0)-4266773472.0)+
                    1072709352.0)/2490808320.0;
      _bet[5] = nx*(_n*(_n*(457888660*_n-312227409)-67920528)+70779852)/
        2490808320.0;
      nx *= _n;
      _alp[6] = nx*(_n*(175214326799.0*_n-171950693600.0)+38652967262.0)/
        58118860800.0;
      _bet[6] = nx*((-19841813847.0*_n-3665348512.0)*_n+3758062126.0)/
        116237721600.0;
      nx *= _n;
      _alp[7] = (13700311101.0-67039739596.0*_n)*nx/12454041600.0;
      _bet[7] = (1979471673.0-1989295244.0*_n)*nx/49816166400.0;
      nx *= _n;
      _alp[8] = 1424729850961.0*nx/743921418240.0;
      _bet[8] = 191773887257.0*nx/3719607091200.0;
      break;
    default:
      STATIC_ASSERT(maxpow_ >= 4 && maxpow_ <= 8, "Bad value of maxpow_");
    }
    // _a1 is the equivalent radius for computing the circumference of
    // ellipse.
    _a1 = _b1 * _a;
  }

  const TransverseMercator
  TransverseMercator::UTM(Constants::WGS84_a<real>(),
                          Constants::WGS84_f<real>(),
                          Constants::UTM_k0<real>());

  // Engsager and Poder (2007) use trigonometric series to convert between phi
  // and phip.

  // Conversion from phi to phip:
  //
  //     phip = phi + sum(c[j] * sin(2*j*phi), j, 1, 6)
  //
  //       c[1] = - 2 * n
  //              + 2/3 * n^2
  //              + 4/3 * n^3
  //              - 82/45 * n^4
  //              + 32/45 * n^5
  //              + 4642/4725 * n^6;
  //       c[2] =   5/3 * n^2
  //              - 16/15 * n^3
  //              - 13/9 * n^4
  //              + 904/315 * n^5
  //              - 1522/945 * n^6;
  //       c[3] = - 26/15 * n^3
  //              + 34/21 * n^4
  //              + 8/5 * n^5
  //              - 12686/2835 * n^6;
  //       c[4] =   1237/630 * n^4
  //              - 12/5 * n^5
  //              - 24832/14175 * n^6;
  //       c[5] = - 734/315 * n^5
  //              + 109598/31185 * n^6;
  //       c[6] =   444337/155925 * n^6;

  // Conversion from phip to phi:
  //
  //     phi = phip + sum(d[j] * sin(2*j*phip), j, 1, 6)
  //
  //       d[1] =   2 * n
  //              - 2/3 * n^2
  //              - 2 * n^3
  //              + 116/45 * n^4
  //              + 26/45 * n^5
  //              - 2854/675 * n^6;
  //       d[2] =   7/3 * n^2
  //              - 8/5 * n^3
  //              - 227/45 * n^4
  //              + 2704/315 * n^5
  //              + 2323/945 * n^6;
  //       d[3] =   56/15 * n^3
  //              - 136/35 * n^4
  //              - 1262/105 * n^5
  //              + 73814/2835 * n^6;
  //       d[4] =   4279/630 * n^4
  //              - 332/35 * n^5
  //              - 399572/14175 * n^6;
  //       d[5] =   4174/315 * n^5
  //              - 144838/6237 * n^6;
  //       d[6] =   601676/22275 * n^6;

  // In order to maintain sufficient relative accuracy close to the pole use
  //
  //     S = sum(c[i]*sin(2*i*phi),i,1,6)
  //     taup = (tau + tan(S)) / (1 - tau * tan(S))

  // taupf and tauf are adapted from TransverseMercatorExact (taup and
  // taupinv).  tau = tan(phi), taup = sinh(psi)
  Math::real TransverseMercator::taupf(real tau) const throw() {
    if (!(abs(tau) < overflow_))
      return tau;
    real
      tau1 = Math::hypot(real(1), tau),
      sig = sinh( eatanhe(tau / tau1) );
    return Math::hypot(real(1), sig) * tau - sig * tau1;
  }

  Math::real TransverseMercator::tauf(real taup) const throw() {
    if (!(abs(taup) < overflow_))
      return taup;
    real
      // To lowest order in e^2, taup = (1 - e^2) * tau = _e2m * tau; so use
      // tau = taup/_e2m as a starting guess.  Only 1 iteration is needed for
      // |lat| < 3.35 deg, otherwise 2 iterations are needed.  If, instead, tau
      // = taup is used the mean number of iterations increases to 1.99 (2
      // iterations are needed except near tau = 0).
      tau = taup/_e2m,
      stol = tol_ * max(real(1), abs(taup));
    // min iterations = 1, max iterations = 2; mean = 1.94
    for (int i = 0; i < numit_; ++i) {
      real
        tau1 = Math::hypot(real(1), tau),
        sig = sinh( eatanhe( tau / tau1 ) ),
        taupa = Math::hypot(real(1), sig) * tau - sig * tau1,
        dtau = (taup - taupa) * (1 + _e2m * Math::sq(tau)) /
        ( _e2m * tau1 * Math::hypot(real(1), taupa) );
      tau += dtau;
      if (!(abs(dtau) >= stol))
        break;
    }
    return tau;
  }

  void TransverseMercator::Forward(real lon0, real lat, real lon,
                                   real& x, real& y, real& gamma, real& k)
    const throw() {
    lon = Math::AngDiff(Math::AngNormalize(lon0), Math::AngNormalize(lon));
    // Explicitly enforce the parity
    int
      latsign = lat < 0 ? -1 : 1,
      lonsign = lon < 0 ? -1 : 1;
    lon *= lonsign;
    lat *= latsign;
    bool backside = lon > 90;
    if (backside) {
      if (lat == 0)
        latsign = -1;
      lon = 180 - lon;
    }
    real
      phi = lat * Math::degree<real>(),
      lam = lon * Math::degree<real>();
    // phi = latitude
    // phi' = conformal latitude
    // psi = isometric latitude
    // tau = tan(phi)
    // tau' = tan(phi')
    // [xi', eta'] = Gauss-Schreiber TM coordinates
    // [xi, eta] = Gauss-Krueger TM coordinates
    //
    // We use
    //   tan(phi') = sinh(psi)
    //   sin(phi') = tanh(psi)
    //   cos(phi') = sech(psi)
    //   denom^2    = 1-cos(phi')^2*sin(lam)^2 = 1-sech(psi)^2*sin(lam)^2
    //   sin(xip)   = sin(phi')/denom          = tanh(psi)/denom
    //   cos(xip)   = cos(phi')*cos(lam)/denom = sech(psi)*cos(lam)/denom
    //   cosh(etap) = 1/denom                  = 1/denom
    //   sinh(etap) = cos(phi')*sin(lam)/denom = sech(psi)*sin(lam)/denom
    real etap, xip;
    if (lat != 90) {
      real
        c = max(real(0), cos(lam)), // cos(pi/2) might be negative
        tau = tan(phi),
        taup = taupf(tau);
      xip = atan2(taup, c);
      // Used to be
      //   etap = Math::atanh(sin(lam) / cosh(psi));
      etap = Math::asinh(sin(lam) / Math::hypot(taup, c));
      // convergence and scale for Gauss-Schreiber TM (xip, etap) -- gamma0 =
      // atan(tan(xip) * tanh(etap)) = atan(tan(lam) * sin(phi'));
      // sin(phi') = tau'/sqrt(1 + tau'^2)
      gamma = atan(tanx(lam) *
                   taup / Math::hypot(real(1), taup)); // Krueger p 22 (44)
      // k0 = sqrt(1 - _e2 * sin(phi)^2) * (cos(phi') / cos(phi)) * cosh(etap)
      // Note 1/cos(phi) = cosh(psip);
      // and cos(phi') * cosh(etap) = 1/hypot(sinh(psi), cos(lam))
      //
      // This form has cancelling errors.  This property is lost if cosh(psip)
      // is replaced by 1/cos(phi), even though it's using "primary" data (phi
      // instead of psip).
      k = sqrt(_e2m + _e2 * Math::sq(cos(phi))) * Math::hypot(real(1), tau)
        / Math::hypot(taup, c);
    } else {
      xip = Math::pi<real>()/2;
      etap = 0;
      gamma = lam;
      k = _c;
    }
    // {xi',eta'} is {northing,easting} for Gauss-Schreiber transverse Mercator
    // (for eta' = 0, xi' = bet). {xi,eta} is {northing,easting} for transverse
    // Mercator with constant scale on the central meridian (for eta = 0, xip =
    // rectifying latitude).  Define
    //
    //   zeta = xi + i*eta
    //   zeta' = xi' + i*eta'
    //
    // The conversion from conformal to rectifying latitude can be expressed as
    // a series in _n:
    //
    //   zeta = zeta' + sum(h[j-1]' * sin(2 * j * zeta'), j = 1..maxpow_)
    //
    // where h[j]' = O(_n^j).  The reversion of this series gives
    //
    //   zeta' = zeta - sum(h[j-1] * sin(2 * j * zeta), j = 1..maxpow_)
    //
    // which is used in Reverse.
    //
    // Evaluate sums via Clenshaw method.  See
    //    http://mathworld.wolfram.com/ClenshawRecurrenceFormula.html
    //
    // Let
    //
    //    S = sum(c[k] * F[k](x), k = 0..N)
    //    F[n+1](x) = alpha(n,x) * F[n](x) + beta(n,x) * F[n-1](x)
    //
    // Evaluate S with
    //
    //    y[N+2] = y[N+1] = 0
    //    y[k] = alpha(k,x) * y[k+1] + beta(k+1,x) * y[k+2] + c[k]
    //    S = c[0] * F[0](x) + y[1] * F[1](x) + beta(1,x) * F[0](x) * y[2]
    //
    // Here we have
    //
    //    x = 2 * zeta'
    //    F[n](x) = sin(n * x)
    //    a(n, x) = 2 * cos(x)
    //    b(n, x) = -1
    //    [ sin(A+B) - 2*cos(B)*sin(A) + sin(A-B) = 0, A = n*x, B = x ]
    //    N = maxpow_
    //    c[k] = _alp[k]
    //    S = y[1] * sin(x)
    //
    // For the derivative we have
    //
    //    x = 2 * zeta'
    //    F[n](x) = cos(n * x)
    //    a(n, x) = 2 * cos(x)
    //    b(n, x) = -1
    //    [ cos(A+B) - 2*cos(B)*cos(A) + cos(A-B) = 0, A = n*x, B = x ]
    //    c[0] = 1; c[k] = 2*k*_alp[k]
    //    S = (c[0] - y[2]) + y[1] * cos(x)
    real
      c0 = cos(2 * xip), ch0 = cosh(2 * etap),
      s0 = sin(2 * xip), sh0 = sinh(2 * etap),
      ar = 2 * c0 * ch0, ai = -2 * s0 * sh0; // 2 * cos(2*zeta')
    int n = maxpow_;
    real
      xi0 = (n & 1 ? _alp[n] : 0), eta0 = 0,
      xi1 = 0, eta1 = 0;
    real                        // Accumulators for dzeta/dzeta'
      yr0 = (n & 1 ? 2 * maxpow_ * _alp[n--] : 0), yi0 = 0,
      yr1 = 0, yi1 = 0;
    while (n) {
      xi1  = ar * xi0 - ai * eta0 - xi1 + _alp[n];
      eta1 = ai * xi0 + ar * eta0 - eta1;
      yr1 = ar * yr0 - ai * yi0 - yr1 + 2 * n * _alp[n];
      yi1 = ai * yr0 + ar * yi0 - yi1;
      --n;
      xi0  = ar * xi1 - ai * eta1 - xi0 + _alp[n];
      eta0 = ai * xi1 + ar * eta1 - eta0;
      yr0 = ar * yr1 - ai * yi1 - yr0 + 2 * n * _alp[n];
      yi0 = ai * yr1 + ar * yi1 - yi0;
      --n;
    }
    ar /= 2; ai /= 2;           // cos(2*zeta')
    yr1 = 1 - yr1 + ar * yr0 - ai * yi0;
    yi1 =   - yi1 + ai * yr0 + ar * yi0;
    ar = s0 * ch0; ai = c0 * sh0; // sin(2*zeta')
    real
      xi  = xip  + ar * xi0 - ai * eta0,
      eta = etap + ai * xi0 + ar * eta0;
    // Fold in change in convergence and scale for Gauss-Schreiber TM to
    // Gauss-Krueger TM.
    gamma -= atan2(yi1, yr1);
    k *= _b1 * Math::hypot(yr1, yi1);
    gamma /= Math::degree<real>();
    y = _a1 * _k0 * (backside ? Math::pi<real>() - xi : xi) * latsign;
    x = _a1 * _k0 * eta * lonsign;
    if (backside)
      gamma = 180 - gamma;
    gamma *= latsign * lonsign;
    k *= _k0;
  }

  void TransverseMercator::Reverse(real lon0, real x, real y,
                                   real& lat, real& lon, real& gamma, real& k)
    const throw() {
    // This undoes the steps in Forward.  The wrinkles are: (1) Use of the
    // reverted series to express zeta' in terms of zeta. (2) Newton's method
    // to solve for phi in terms of tan(phi).
    real
      xi = y / (_a1 * _k0),
      eta = x / (_a1 * _k0);
    // Explicitly enforce the parity
    int
      xisign = xi < 0 ? -1 : 1,
      etasign = eta < 0 ? -1 : 1;
    xi *= xisign;
    eta *= etasign;
    bool backside = xi > Math::pi<real>()/2;
    if (backside)
      xi = Math::pi<real>() - xi;
    real
      c0 = cos(2 * xi), ch0 = cosh(2 * eta),
      s0 = sin(2 * xi), sh0 = sinh(2 * eta),
      ar = 2 * c0 * ch0, ai = -2 * s0 * sh0; // 2 * cos(2*zeta)
    int n = maxpow_;
    real                        // Accumulators for zeta'
      xip0 = (n & 1 ? -_bet[n] : 0), etap0 = 0,
      xip1 = 0, etap1 = 0;
    real                        // Accumulators for dzeta'/dzeta
      yr0 = (n & 1 ? - 2 * maxpow_ * _bet[n--] : 0), yi0 = 0,
      yr1 = 0, yi1 = 0;
    while (n) {
      xip1  = ar * xip0 - ai * etap0 - xip1 - _bet[n];
      etap1 = ai * xip0 + ar * etap0 - etap1;
      yr1 = ar * yr0 - ai * yi0 - yr1 - 2 * n * _bet[n];
      yi1 = ai * yr0 + ar * yi0 - yi1;
      --n;
      xip0  = ar * xip1 - ai * etap1 - xip0 - _bet[n];
      etap0 = ai * xip1 + ar * etap1 - etap0;
      yr0 = ar * yr1 - ai * yi1 - yr0 - 2 * n * _bet[n];
      yi0 = ai * yr1 + ar * yi1 - yi0;
      --n;
    }
    ar /= 2; ai /= 2;           // cos(2*zeta')
    yr1 = 1 - yr1 + ar * yr0 - ai * yi0;
    yi1 =   - yi1 + ai * yr0 + ar * yi0;
    ar = s0 * ch0; ai = c0 * sh0; // sin(2*zeta)
    real
      xip  = xi  + ar * xip0 - ai * etap0,
      etap = eta + ai * xip0 + ar * etap0;
    // Convergence and scale for Gauss-Schreiber TM to Gauss-Krueger TM.
    gamma = atan2(yi1, yr1);
    k = _b1 / Math::hypot(yr1, yi1);
    // JHS 154 has
    //
    //   phi' = asin(sin(xi') / cosh(eta')) (Krueger p 17 (25))
    //   lam = asin(tanh(eta') / cos(phi')
    //   psi = asinh(tan(phi'))
    real lam, phi;
    real
      s = sinh(etap),
      c = max(real(0), cos(xip)), // cos(pi/2) might be negative
      r = Math::hypot(s, c);
    if (r != 0) {
      lam = atan2(s, c);        // Krueger p 17 (25)
      // Use Newton's method to solve for tau
      real
        taup = sin(xip)/r,
        tau = tauf(taup);
      phi = atan(tau);
      gamma += atan(tanx(xip) * tanh(etap)); // Krueger p 19 (31)
      // Note cos(phi') * cosh(eta') = r
      k *= sqrt(_e2m + _e2 * Math::sq(cos(phi))) *
        Math::hypot(real(1), tau) * r;
    } else {
      phi = Math::pi<real>()/2;
      lam = 0;
      k *= _c;
    }
    lat = phi / Math::degree<real>() * xisign;
    lon = lam / Math::degree<real>();
    if (backside)
      lon = 180 - lon;
    lon *= etasign;
    lon = Math::AngNormalize(lon + Math::AngNormalize(lon0));
    gamma /= Math::degree<real>();
    if (backside)
      gamma = 180 - gamma;
    gamma *= xisign * etasign;
    k *= _k0;
  }

} // namespace GeographicLib

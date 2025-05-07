/**
 * \file signtest.cpp
 * \brief Test treatment of +/-0 and +/-180
 *
 * Copyright (c) Charles Karney (2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <iostream>
#include <limits>
#include <string>
#include <GeographicLib/Math.hpp>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/PolygonArea.hpp>

// On Centos 7, remquo(810.0, 90.0 &q) returns 90.0 with q=8.  Rather than
// lousing up Math.cpp with this problem we just skip the failing tests.
#if defined(__GNUG__) && __GNUG__ < 11
#  define BUGGY_REMQUO 1
#else
#  define BUGGY_REMQUO 0
#endif

// Visual Studio C++ does not implement round to even, see
// https://developercommunity.visualstudio.com/t/stdfixed-output-does-not-implement-round-to-even/1671088
// reported on 2022-02-20.  Problem can't be reproduced at Microsoft, probably
// this is some issue with the runtime library.
//
// Broken on my Windows desktop system vc14 thru vc16
// Fixed on my Windows laptop system vc16 and vc17
// Broken on build-open machines, vc14 and vc15+win32
// Fixed on build-open machines, vc15+x64 and vc16
// Let's assume that it's OK for vc16 and later
#if defined(_MSC_VER) && _MSC_VER < 1920
#  define BUGGY_ROUNDING 1
#else
#  define BUGGY_ROUNDING 0
#endif

// use "do { } while (false)" idiom so it can be punctuated like a statement.
#define REMQUO_CHECK(CMD) do { if (!BUGGY_REMQUO) { CMD; } } while (false)
#define ROUNDING_CHECK(CMD) do { if (!BUGGY_ROUNDING) { CMD; } } while (false)

using namespace std;
using namespace GeographicLib;

typedef Math::real T;

static int equiv(T x, T y) {
  using std::isnan;             // Needed for Centos 7, ubuntu 14
  return ( (isnan(x) && isnan(y)) || (x == y && signbit(x) == signbit(y)) ) ?
    0 : 1;
}

static int checkEquals(T x, T y, T d) {
  if (fabs(x - y) <= d)
    return 0;
  cout << "checkEquals fails: " << x << " != " << y << " +/- " << d << "\n";
  return 1;
}

#define check(expr, r) do {                         \
    T s = T(r),  t = expr;                          \
    if (equiv(s, t)) {                              \
      cout << "Line " << __LINE__ << ": " << #expr  \
           << " != " << #r << " (" << t << ")\n";   \
      ++n;                                          \
    }                                               \
  } while (false)

#define checksincosd(x, s, c) do {                  \
    T sx, cx;                                       \
    Math::sincosd(x, sx, cx);                       \
    if (equiv(s, sx)) {                             \
      cout << "Line " << __LINE__ << ": sin(" << x  \
           << ") != " << s << " (" << sx << ")\n";  \
      ++n;                                          \
    }                                               \
    if (equiv(c, cx)) {                             \
      cout << "Line " << __LINE__ << ": cos(" << x  \
           << ") != " << c << " (" << cx << ")\n";  \
      ++n;                                          \
    }                                               \
  } while (false)

#define strcheck(expr, r) do {                     \
    string s = string(r),  ss = expr;              \
    if (!(s == ss)) {                              \
      cout << "Line " << __LINE__ << ": " << #expr \
           << " != " << #r << " (" << ss << ")\n"; \
      ++n;                                         \
    }                                              \
  } while (false)

int main() {
  T inf = Math::infinity(),
    nan = Math::NaN(),
    eps = numeric_limits<T>::epsilon(),
    ovf = 1 / Math::sq(eps),
    e;
  int n = 0;

  check( Math::AngRound(-eps/32), -eps/32);
  check( Math::AngRound(-eps/64), -0.0   );
  check( Math::AngRound(-  T(0)), -0.0   );
  check( Math::AngRound(   T(0)), +0.0   );
  check( Math::AngRound( eps/64), +0.0   );
  check( Math::AngRound( eps/32), +eps/32);
  check( Math::AngRound((1-2*eps)/64), (1-2*eps)/64);
  check( Math::AngRound((1-eps  )/64),  T(1)    /64);
  check( Math::AngRound((1-eps/2)/64),  T(1)    /64);
  check( Math::AngRound((1-eps/4)/64),  T(1)    /64);
  check( Math::AngRound( T(1)    /64),  T(1)    /64);
  check( Math::AngRound((1+eps/2)/64),  T(1)    /64);
  check( Math::AngRound((1+eps  )/64),  T(1)    /64);
  check( Math::AngRound((1+2*eps)/64), (1+2*eps)/64);
  check( Math::AngRound((1-eps  )/32), (1-eps  )/32);
  check( Math::AngRound((1-eps/2)/32),  T(1)    /32);
  check( Math::AngRound((1-eps/4)/32),  T(1)    /32);
  check( Math::AngRound( T(1)    /32),  T(1)    /32);
  check( Math::AngRound((1+eps/2)/32),  T(1)    /32);
  check( Math::AngRound((1+eps  )/32), (1+eps  )/32);
  check( Math::AngRound((1-eps  )/16), (1-eps  )/16);
  check( Math::AngRound((1-eps/2)/16), (1-eps/2)/16);
  check( Math::AngRound((1-eps/4)/16),  T(1)    /16);
  check( Math::AngRound( T(1)    /16),  T(1)    /16);
  check( Math::AngRound((1+eps/4)/16),  T(1)    /16);
  check( Math::AngRound((1+eps/2)/16),  T(1)    /16);
  check( Math::AngRound((1+eps  )/16), (1+eps  )/16);
  check( Math::AngRound((1-eps  )/ 8), (1-eps  )/ 8);
  check( Math::AngRound((1-eps/2)/ 8), (1-eps/2)/ 8);
  check( Math::AngRound((1-eps/4)/ 8),  T(1)    / 8);
  check( Math::AngRound((1+eps/2)/ 8),  T(1)    / 8);
  check( Math::AngRound((1+eps  )/ 8), (1+eps  )/ 8);
  check( Math::AngRound( 1-eps      ),  1-eps      );
  check( Math::AngRound( 1-eps/2    ),  1-eps/2    );
  check( Math::AngRound( 1-eps/4    ),  1          );
  check( Math::AngRound( T(1)       ),  1          );
  check( Math::AngRound( 1+eps/4    ),  1          );
  check( Math::AngRound( 1+eps/2    ),  1          );
  check( Math::AngRound( 1+eps      ),  1+  eps    );
  check( Math::AngRound(T(90)-64*eps),  90-64*eps  );
  check( Math::AngRound(T(90)-32*eps),  90         );
  check( Math::AngRound(T(90)       ),  90         );

  checksincosd(-  inf ,  nan,  nan);
  REMQUO_CHECK( checksincosd(-T(810), -1.0, +0.0) );
  checksincosd(-T(720), -0.0, +1.0);
  checksincosd(-T(630), +1.0, +0.0);
  checksincosd(-T(540), -0.0, -1.0);
  checksincosd(-T(450), -1.0, +0.0);
  checksincosd(-T(360), -0.0, +1.0);
  checksincosd(-T(270), +1.0, +0.0);
  checksincosd(-T(180), -0.0, -1.0);
  checksincosd(-T( 90), -1.0, +0.0);
  checksincosd(-T(  0), -0.0, +1.0);
  checksincosd(+T(  0), +0.0, +1.0);
  checksincosd(+T( 90), +1.0, +0.0);
  checksincosd(+T(180), +0.0, -1.0);
  checksincosd(+T(270), -1.0, +0.0);
  checksincosd(+T(360), +0.0, +1.0);
  checksincosd(+T(450), +1.0, +0.0);
  checksincosd(+T(540), +0.0, -1.0);
  checksincosd(+T(630), -1.0, +0.0);
  checksincosd(+T(720), +0.0, +1.0);
  REMQUO_CHECK( checksincosd(+T(810), +1.0, +0.0) );
  checksincosd(+  inf ,  nan,  nan);
  checksincosd(   nan ,  nan,  nan);

  {
    T s1, c1, s2, c2, s3, c3;
    Math::sincosd(T(         9), s1, c1);
    Math::sincosd(T(        81), s2, c2);
    Math::sincosd(T(-123456789), s3, c3);
    int j = equiv(s1, c2) + equiv(c1, s2);
    REMQUO_CHECK( j += equiv(s1, s3) + equiv(c1, -c3) );
    if (j) {
      cout << "Line " << __LINE__ << " : sincos accuracy fail\n";
      ++n;
    }
  }

  check( Math::sind(-  inf ),  nan);
  check( Math::sind(-T(720)), -0.0);
  check( Math::sind(-T(540)), -0.0);
  check( Math::sind(-T(360)), -0.0);
  check( Math::sind(-T(180)), -0.0);
  check( Math::sind(-T(  0)), -0.0);
  check( Math::sind(+T(  0)), +0.0);
  check( Math::sind(+T(180)), +0.0);
  check( Math::sind(+T(360)), +0.0);
  check( Math::sind(+T(540)), +0.0);
  check( Math::sind(+T(720)), +0.0);
  check( Math::sind(+  inf ),  nan);

  check( Math::cosd(-  inf ),  nan);
  REMQUO_CHECK( check( Math::cosd(-T(810)), +0.0) );
  check( Math::cosd(-T(630)), +0.0);
  check( Math::cosd(-T(450)), +0.0);
  check( Math::cosd(-T(270)), +0.0);
  check( Math::cosd(-T( 90)), +0.0);
  check( Math::cosd(+T( 90)), +0.0);
  check( Math::cosd(+T(270)), +0.0);
  check( Math::cosd(+T(450)), +0.0);
  check( Math::cosd(+T(630)), +0.0);
  REMQUO_CHECK( check( Math::cosd(+T(810)), +0.0) );
  check( Math::cosd(+  inf ),  nan);

#if !(defined(_MSC_VER) && _MSC_VER <= 1900)
  check( Math::tand(-  inf ),  nan);
#endif
  REMQUO_CHECK( check( Math::tand(-T(810)), -ovf) );
  check( Math::tand(-T(720)), -0.0);
  check( Math::tand(-T(630)), +ovf);
  check( Math::tand(-T(540)), +0.0);
  check( Math::tand(-T(450)), -ovf);
  check( Math::tand(-T(360)), -0.0);
  check( Math::tand(-T(270)), +ovf);
  check( Math::tand(-T(180)), +0.0);
  check( Math::tand(-T( 90)), -ovf);
  check( Math::tand(-T(  0)), -0.0);
  check( Math::tand(+T(  0)), +0.0);
  check( Math::tand(+T( 90)), +ovf);
  check( Math::tand(+T(180)), -0.0);
  check( Math::tand(+T(270)), -ovf);
  check( Math::tand(+T(360)), +0.0);
  check( Math::tand(+T(450)), +ovf);
  check( Math::tand(+T(540)), -0.0);
  check( Math::tand(+T(630)), -ovf);
  check( Math::tand(+T(720)), +0.0);
  REMQUO_CHECK( check( Math::tand(+T(810)), +ovf) );
#if !(defined(_MSC_VER) && _MSC_VER <= 1900)
  check( Math::tand(+  inf ),  nan);
#endif

  check( Math::atan2d(+T(0), -T(0)), +180 );
  check( Math::atan2d(-T(0), -T(0)), -180 );
  check( Math::atan2d(+T(0), +T(0)), +0.0 );
  check( Math::atan2d(-T(0), +T(0)), -0.0 );
  check( Math::atan2d(+T(0), -T(1)), +180 );
  check( Math::atan2d(-T(0), -T(1)), -180 );
  check( Math::atan2d(+T(0), +T(1)), +0.0 );
  check( Math::atan2d(-T(0), +T(1)), -0.0 );
  check( Math::atan2d(-T(1), +T(0)),  -90 );
  check( Math::atan2d(-T(1), -T(0)),  -90 );
  check( Math::atan2d(+T(1), +T(0)),  +90 );
  check( Math::atan2d(+T(1), -T(0)),  +90 );
  check( Math::atan2d(+T(1),  -inf), +180 );
  check( Math::atan2d(-T(1),  -inf), -180 );
  check( Math::atan2d(+T(1),  +inf), +0.0 );
  check( Math::atan2d(-T(1),  +inf), -0.0 );
  check( Math::atan2d( +inf, +T(1)),  +90 );
  check( Math::atan2d( +inf, -T(1)),  +90 );
  check( Math::atan2d( -inf, +T(1)),  -90 );
  check( Math::atan2d( -inf, -T(1)),  -90 );
  check( Math::atan2d( +inf,  -inf), +135 );
  check( Math::atan2d( -inf,  -inf), -135 );
  check( Math::atan2d( +inf,  +inf),  +45 );
  check( Math::atan2d( -inf,  +inf),  -45 );
  check( Math::atan2d(  nan, +T(1)),  nan );
  check( Math::atan2d(+T(1),   nan),  nan );

  {
    T s = 7e-16;
    if ( equiv( Math::atan2d(s, -T(1)), 180 - Math::atan2d(s, T(1)) ) ) {
      cout << "Line " << __LINE__ << " : atan2d accuracy fail\n";
      ++n;
    }
  }

  check( Math::sum(+T(9), -T(9), e), +0.0 );
  check( Math::sum(-T(9), +T(9), e), +0.0 );
  check( Math::sum(-T(0), +T(0), e), +0.0 );
  check( Math::sum(+T(0), -T(0), e), +0.0 );
  check( Math::sum(-T(0), -T(0), e), -0.0 );
  check( Math::sum(+T(0), +T(0), e), +0.0 );

  check( Math::AngNormalize(-T(900)), -180 );
  check( Math::AngNormalize(-T(720)), -0.0 );
  check( Math::AngNormalize(-T(540)), -180 );
  check( Math::AngNormalize(-T(360)), -0.0 );
  check( Math::AngNormalize(-T(180)), -180 );
  check( Math::AngNormalize(  -T(0)), -0.0 );
  check( Math::AngNormalize(  +T(0)), +0.0 );
  check( Math::AngNormalize( T(180)), +180 );
  check( Math::AngNormalize( T(360)), +0.0 );
  check( Math::AngNormalize( T(540)), +180 );
  check( Math::AngNormalize( T(720)), +0.0 );
  check( Math::AngNormalize( T(900)), +180 );

  check( Math::AngDiff(+T(  0), +T(  0), e), +0.0 );
  check( Math::AngDiff(+T(  0), -T(  0), e), -0.0 );
  check( Math::AngDiff(-T(  0), +T(  0), e), +0.0 );
  check( Math::AngDiff(-T(  0), -T(  0), e), +0.0 );
  check( Math::AngDiff(+T(  5), +T(365), e), +0.0 );
  check( Math::AngDiff(+T(365), +T(  5), e), -0.0 );
  check( Math::AngDiff(+T(  5), +T(185), e), +180.0 );
  check( Math::AngDiff(+T(185), +T(  5), e), -180.0 );
  check( Math::AngDiff( +eps  , +T(180), e), +180.0 );
  check( Math::AngDiff( -eps  , +T(180), e), -180.0 );
  check( Math::AngDiff( +eps  , -T(180), e), +180.0 );
  check( Math::AngDiff( -eps  , -T(180), e), -180.0 );

  {
    T x = 138 + 128 * eps, y = -164;
    if ( equiv( Math::AngDiff(x, y), 58 - 128 * eps ) ) {
      cout << "Line " << __LINE__ << " : AngDiff accuracy fail\n";
      ++n;
    }
  }

  check( Utility::val<T>("+0"), +0.0 );
  check( Utility::val<T>("-0"), -0.0 );
  check( Utility::val<T>("nan"), nan );
  check( Utility::val<T>("+inf"), +inf );
  check( Utility::val<T>( "inf"), +inf );
  check( Utility::val<T>("-inf"), -inf );

  strcheck( Utility::str<T>( nan, 0),  "nan" );
  strcheck( Utility::str<T>(-inf, 0), "-inf" );
  strcheck( Utility::str<T>(+inf, 0),  "inf" );
  strcheck( Utility::str<T>(-T(3.5), 0),   "-4" );
  ROUNDING_CHECK( strcheck( Utility::str<T>(-T(2.5), 0),   "-2" ) );
  strcheck( Utility::str<T>(-T(1.5), 0),   "-2" );
  ROUNDING_CHECK( strcheck( Utility::str<T>(-T(0.5), 0),   "-0" ) );
  strcheck( Utility::str<T>(-T(0  ), 0),   "-0" );
  strcheck( Utility::str<T>(+T(0  ), 0),    "0" );
  ROUNDING_CHECK( strcheck( Utility::str<T>(+T(0.5), 0),    "0" ) );
  strcheck( Utility::str<T>(+T(1.5), 0),    "2" );
  ROUNDING_CHECK( strcheck( Utility::str<T>(+T(2.5), 0),    "2" ) );
  strcheck( Utility::str<T>(+T(3.5), 0),    "4" );
  strcheck( Utility::str<T>(-T(1.75), 1), "-1.8");
  ROUNDING_CHECK( strcheck( Utility::str<T>(-T(1.25), 1), "-1.2") );
  strcheck( Utility::str<T>(-T(0.75), 1), "-0.8");
  ROUNDING_CHECK( strcheck( Utility::str<T>(-T(0.25), 1), "-0.2") );
  strcheck( Utility::str<T>(-T(0   ), 1), "-0.0");
  strcheck( Utility::str<T>(+T(0   ), 1),  "0.0");
  ROUNDING_CHECK( strcheck( Utility::str<T>(+T(0.25), 1),  "0.2") );
  strcheck( Utility::str<T>(+T(0.75), 1),  "0.8");
  ROUNDING_CHECK( strcheck( Utility::str<T>(+T(1.25), 1),  "1.2") );
  strcheck( Utility::str<T>(+T(1.75), 1),  "1.8");

  strcheck( DMS::Encode( nan, DMS::DEGREE, 0),  "nan" );
  strcheck( DMS::Encode(-inf, DMS::DEGREE, 0), "-inf" );
  strcheck( DMS::Encode(+inf, DMS::DEGREE, 0),  "inf" );
  strcheck( DMS::Encode(-T(3.5), DMS::DEGREE, 0),   "-4" );
  ROUNDING_CHECK( strcheck( DMS::Encode(-T(2.5), DMS::DEGREE, 0),   "-2" ) );
  strcheck( DMS::Encode(-T(1.5), DMS::DEGREE, 0),   "-2" );
  ROUNDING_CHECK( strcheck( DMS::Encode(-T(0.5), DMS::DEGREE, 0),   "-0" ) );
  strcheck( DMS::Encode(-T(0  ), DMS::DEGREE, 0),   "-0" );
  strcheck( DMS::Encode(+T(0  ), DMS::DEGREE, 0),    "0" );
  ROUNDING_CHECK( strcheck( DMS::Encode(+T(0.5), DMS::DEGREE, 0),    "0" ) );
  strcheck( DMS::Encode(+T(1.5), DMS::DEGREE, 0),    "2" );
  ROUNDING_CHECK( strcheck( DMS::Encode(+T(2.5), DMS::DEGREE, 0),    "2" ) );
  strcheck( DMS::Encode(+T(3.5), DMS::DEGREE, 0),    "4" );
  strcheck( DMS::Encode(-T(1.75), DMS::DEGREE, 1), "-1.8");
  ROUNDING_CHECK( strcheck( DMS::Encode(-T(1.25), DMS::DEGREE, 1), "-1.2") );
  strcheck( DMS::Encode(-T(0.75), DMS::DEGREE, 1), "-0.8");
  ROUNDING_CHECK( strcheck( DMS::Encode(-T(0.25), DMS::DEGREE, 1), "-0.2") );
  strcheck( DMS::Encode(-T(0   ), DMS::DEGREE, 1), "-0.0");
  strcheck( DMS::Encode(+T(0   ), DMS::DEGREE, 1),  "0.0");
  ROUNDING_CHECK( strcheck( DMS::Encode(+T(0.25), DMS::DEGREE, 1),  "0.2") );
  strcheck( DMS::Encode(+T(0.75), DMS::DEGREE, 1),  "0.8");
  ROUNDING_CHECK( strcheck( DMS::Encode(+T(1.25), DMS::DEGREE, 1),  "1.2") );
  strcheck( DMS::Encode(+T(1.75), DMS::DEGREE, 1),  "1.8");
  strcheck( DMS::Encode( T(1e20), DMS::DEGREE, 0), "100000000000000000000");
  strcheck( DMS::Encode( T(1e21), DMS::DEGREE, 0), "1000000000000000000000");
  {
    T t = -(1 + 2/T(60) + T(2.99)/3600);

    strcheck( DMS::Encode( t,DMS::DEGREE,0,DMS::NONE     ), "-1"         );
    strcheck( DMS::Encode( t,DMS::DEGREE,0,DMS::LATITUDE ), "01S"         );
    strcheck( DMS::Encode( t,DMS::DEGREE,0,DMS::LONGITUDE),"001W"         );
    strcheck( DMS::Encode(-t,DMS::DEGREE,0,DMS::AZIMUTH  ),"001"          );
    strcheck( DMS::Encode( t,DMS::DEGREE,1,DMS::NONE     ), "-1.0"       );
    strcheck( DMS::Encode( t,DMS::DEGREE,1,DMS::LATITUDE ), "01.0S"       );
    strcheck( DMS::Encode( t,DMS::DEGREE,1,DMS::LONGITUDE),"001.0W"       );
    strcheck( DMS::Encode(-t,DMS::DEGREE,1,DMS::AZIMUTH  ),"001.0"        );
    strcheck( DMS::Encode( t,DMS::MINUTE,0,DMS::NONE     ), "-1d02'"      );
    strcheck( DMS::Encode( t,DMS::MINUTE,0,DMS::LATITUDE ), "01d02'S"      );
    strcheck( DMS::Encode( t,DMS::MINUTE,0,DMS::LONGITUDE),"001d02'W"      );
    strcheck( DMS::Encode(-t,DMS::MINUTE,0,DMS::AZIMUTH  ),"001d02'"       );
    strcheck( DMS::Encode( t,DMS::MINUTE,1,DMS::NONE     ), "-1d02.0'"    );
    strcheck( DMS::Encode( t,DMS::MINUTE,1,DMS::LATITUDE ), "01d02.0'S"    );
    strcheck( DMS::Encode( t,DMS::MINUTE,1,DMS::LONGITUDE),"001d02.0'W"    );
    strcheck( DMS::Encode(-t,DMS::MINUTE,1,DMS::AZIMUTH  ),"001d02.0'"     );
    strcheck( DMS::Encode( t,DMS::SECOND,0,DMS::NONE     ), "-1d02'03\""  );
    strcheck( DMS::Encode( t,DMS::SECOND,0,DMS::LATITUDE ), "01d02'03\"S"  );
    strcheck( DMS::Encode( t,DMS::SECOND,0,DMS::LONGITUDE),"001d02'03\"W"  );
    strcheck( DMS::Encode(-t,DMS::SECOND,0,DMS::AZIMUTH  ),"001d02'03\""   );
    strcheck( DMS::Encode( t,DMS::SECOND,1,DMS::NONE     ), "-1d02'03.0\"");
    strcheck( DMS::Encode( t,DMS::SECOND,1,DMS::LATITUDE ), "01d02'03.0\"S");
    strcheck( DMS::Encode( t,DMS::SECOND,1,DMS::LONGITUDE),"001d02'03.0\"W");
    strcheck( DMS::Encode(-t,DMS::SECOND,1,DMS::AZIMUTH  ),"001d02'03.0\"" );
  }
  DMS::flag ind;
  check( DMS::Decode(" +0 ", ind),  +0.0 );
  check( DMS::Decode("-0  ", ind),  -0.0 );
  check( DMS::Decode(" nan", ind),   nan );
  check( DMS::Decode("+inf", ind),  +inf );
  check( DMS::Decode(" inf", ind),  +inf );
  check( DMS::Decode("-inf", ind),  -inf );
  check( DMS::Decode(" +0N", ind),  +0.0 );
  check( DMS::Decode("-0N ", ind),  -0.0 );
  check( DMS::Decode("+0S ", ind),  -0.0 );
  check( DMS::Decode(" -0S", ind),  +0.0 );

  {
    // azimuth of geodesic line with points on equator determined by signs of
    // latitude
    // lat1 lat2 azi1/2
    T C[2][3] = {
      { +T(0), -T(0), 180 },
      { -T(0), +T(0),   0 }
    };
    const Geodesic& g = Geodesic::WGS84();
    const GeodesicExact& ge = GeodesicExact::WGS84();
    T azi1, azi2;
    int i = 0;
    for (int k = 0; k < 2; ++k) {
      g.Inverse(C[k][0], T(0), C[k][1], T(0), azi1, azi2);
      if ( equiv(azi1, C[k][2]) + equiv(azi2, C[k][2]) ) ++i;
      ge.Inverse(C[k][0], T(0), C[k][1], T(0), azi1, azi2);
      if ( equiv(azi1, C[k][2]) + equiv(azi2, C[k][2]) ) ++i;
    }
    if (i) {
      cout << "Line " << __LINE__
           << ": Geodesic::Inverse coincident points on equator fail\n";
      ++n;
    }
  }

  {
    // Does the nearly antipodal equatorial solution go north or south?
    // lat1 lat2 azi1 azi2
    T C[2][4] = {
      { +T(0), +T(0),  56, 124},
      { -T(0), -T(0), 124,  56}
    };
    const Geodesic& g = Geodesic::WGS84();
    const GeodesicExact& ge = GeodesicExact::WGS84();
    T azi1, azi2;
    int i = 0;
    for (int k = 0; k < 2; ++k) {
      g.Inverse(C[k][0], T(0), C[k][1], T(179.5), azi1, azi2);
      i += checkEquals(azi1, C[k][2], 1) + checkEquals(azi2, C[k][3], 1);
      ge.Inverse(C[k][0], T(0), C[k][1], T(179.5), azi1, azi2);
      i += checkEquals(azi1, C[k][2], 1) + checkEquals(azi2, C[k][3], 1);
    }
    if (i) {
      cout << "Line " << __LINE__
           << ": Geodesic::Inverse nearly antipodal points on equator fail\n";
      ++n;
    }
  }

  {
    // How does the exact antipodal equatorial path go N/S + E/W
    // lat1 lat2 lon2 azi1 azi2
    T C[4][5] = {
      { +T(0), +T(0), +180,   +T(0), +180},
      { -T(0), -T(0), +180, +180,   +T(0)},
      { +T(0), +T(0), -180,   -T(0), -180},
      { -T(0), -T(0), -180, -180,   -T(0)}
    };
    const Geodesic& g = Geodesic::WGS84();
    const GeodesicExact& ge = GeodesicExact::WGS84();
    T azi1, azi2;
    int i = 0;
    for (int k = 0; k < 4; ++k) {
      g.Inverse(C[k][0], T(0), C[k][1], C[k][2], azi1, azi2);
      if ( equiv(azi1, C[k][3]) + equiv(azi2, C[k][4]) ) ++i;
      ge.Inverse(C[k][0], T(0), C[k][1], C[k][2], azi1, azi2);
      if ( equiv(azi1, C[k][3]) + equiv(azi2, C[k][4]) ) ++i;
    }
    if (i) {
      cout << "Line " << __LINE__
           << ": Geodesic::Inverse antipodal points on equator fail\n";
      ++n;
    }
  }

  {
    // Antipodal points on the equator with prolate ellipsoid
    // lon2 azi1/2
    T C[2][2] = {
      { +180, +90 },
      { -180, -90 }
    };
    const Geodesic g(T(6.4e6), -1/T(300));
    const GeodesicExact ge(T(6.4e6), -1/T(300));
    T azi1, azi2;
    int i = 0;
    for (int k = 0; k < 2; ++k) {
      g.Inverse(T(0), T(0), T(0), C[k][0], azi1, azi2);
      if ( equiv(azi1, C[k][1]) + equiv(azi2, C[k][1]) ) ++i;
      ge.Inverse(T(0), T(0), T(0), C[k][0], azi1, azi2);
      if ( equiv(azi1, C[k][1]) + equiv(azi2, C[k][1]) ) ++i;
    }
    if (i) {
      cout << "Line " << __LINE__
           << ": Geodesic::Inverse antipodal points on equator, prolate, fail\n";
      ++n;
    }
  }

  {
    // azimuths = +/-0 and +/-180 for the direct problem
    // azi1, lon2, azi2
    T C[4][3] = {
      { +T(0), +180, +180  },
      { -T(0), -180, -180  },
      { +180 , +180, +T(0) },
      { -180 , -180, -T(0) }
    };
    const Geodesic& g = Geodesic::WGS84();
    const GeodesicExact& ge = GeodesicExact::WGS84();
    T lon2, azi2;
    int i = 0;
    for (int k = 0; k < 4; ++k) {
      T t;
      g.GenDirect(T(0), T(0), C[k][0], false, T(15e6),
                  Geodesic::LONGITUDE | Geodesic::AZIMUTH |
                  Geodesic::LONG_UNROLL,
                  t, lon2, azi2,
                  t, t, t, t, t);
      if ( equiv(lon2, C[k][1]) + equiv(azi2, C[k][2]) ) ++i;
      ge.GenDirect(T(0), T(0), C[k][0], false, T(15e6),
                   Geodesic::LONGITUDE | Geodesic::AZIMUTH |
                   Geodesic::LONG_UNROLL,
                   t, lon2, azi2,
                   t, t, t, t, t);
      if ( equiv(lon2, C[k][1]) + equiv(azi2, C[k][2]) ) ++i;
    }
    if (i) {
      cout << "Line " << __LINE__
           << ": Geodesic::Direct azi1 = +/-0 +/-180, fail\n";
      ++n;
    }
  }

  {
    // lat = +/-0 in UTMUPS::Forward
    // lat y northp
    T C[2][3] = {
      { +T(0), T(0), 1 },
      { -T(0), 10e6, 0 }
    };
    int i = 0;
    bool northp; int zone; T x, y; string mgrs;
    for (int k = 0; k < 2; ++k) {
      UTMUPS::Forward(C[k][0], T(3), zone, northp, x, y);
      if ( equiv(y, C[k][1]) + (northp == (C[k][2] > 0) ? 0 : 1) ) ++i;
      MGRS::Forward(zone, northp, x, y, 2, mgrs);
      if (!( mgrs == (k == 0 ? "31NEA0000" : "31MEV0099") )) ++i;
      MGRS::Forward(zone, northp, x, y, +T(0), 2, mgrs);
      if (!( mgrs == (k == 0 ? "31NEA0000" : "31MEV0099") )) ++i;
      MGRS::Forward(zone, northp, x, y, -T(0), 2, mgrs);
      if (!( mgrs == (k == 0 ? "31NEA0000" : "31MEV0099") )) ++i;
    }
    if (i) {
      cout << "Line " << __LINE__
           << ": UTMUPS/MGRS::Forward lat = +/-0, fail\n";
      ++n;
    }
  }

  {
    // Implement check on AddEdge bug: AddPoint(0,0) +
    // AddEdge(90, 1000) + AddEdge(0, 1000) + AddEdge(-90, 0).  The area
    // should be 1e6.  Prior to the fix it was 1e6 - A/2, where A = ellipsoid
    // area.
    // add_test (NAME Planimeter29 COMMAND Planimeter ...)
    const Geodesic& g = Geodesic::WGS84();
    PolygonArea polygon(g);
    polygon.AddPoint(0, 0);
    polygon.AddEdge( 90, 1000);
    polygon.AddEdge(  0, 1000);
    polygon.AddEdge(-90, 1000);
    T perim, area;
    // The area should be 1e6.  Prior to the fix it was 1e6 - A/2, where
    // A = ellipsoid area.
    polygon.Compute(false, true, perim, area);
    int i = checkEquals(area, 1000000, 0.01);
    if (i)  {
      cout << "Line " << __LINE__
           << ": Planimeter29 fail\n";
      ++n;
    }
  }

  if (n) {
    cout << n << " failure" << (n > 1 ? "s" : "") << "\n";
    return 1;
  }
}

/**
 * \file Intersect.cpp
 * \brief Implementation for GeographicLib::Intersect class
 *
 * Copyright (c) Charles Karney (2023) <karney@alum.mit.edu> and licensed under
 * the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/Intersect.hpp>
#include <limits>
#include <utility>
#include <algorithm>
#include <set>

using namespace std;

namespace GeographicLib {

  Intersect::Intersect(const Geodesic& geod)
    : _geod(geod)
    , _a(_geod.EquatorialRadius())
    , _f(_geod.Flattening())
    , _rR(sqrt(_geod.EllipsoidArea() / (4 * Math::pi())))
    , _d(_rR * Math::pi())      // Used to normalize intersection points
    , _eps(3 * numeric_limits<real>::epsilon())
    , _tol(_d * pow(numeric_limits<real>::epsilon(), 3/real(4)))
    , _delta(_d * pow(numeric_limits<real>::epsilon(), 1/real(5)))
    , _comp(_delta)
    , _cnt0(0)
    , _cnt1(0)
    , _cnt2(0)
    , _cnt3(0)
    , _cnt4(0)
  {
    _t1 = _t4 = _a * (1 - _f) * Math::pi();
    _t2 = 2 * distpolar(90);
    _geod.Inverse(0, 0, 90, 0, _t5); _t5 *= 2;
    if (_f > 0) {
      _t3 = distoblique();
      _t4 = _t1;
    } else {
      _t3 = _t5;
      _t4 = polarb();
      swap(_t1, _t2);
    }
    _d1 = _t2 / 2;
    _d2 = 2 * _t3 / 3;
    _d3 = _t4 - _delta;
    if (! (_d1 < _d3 && _d2 < _d3 && _d2 < 2 * _t1) )
      throw GeographicErr("Ellipsoid too eccentric for Closest");
  }

  Intersect::Point
  Intersect::Closest(Math::real latX, Math::real lonX, Math::real aziX,
                     Math::real latY, Math::real lonY, Math::real aziY,
                     const Intersect::Point& p0, int* c) const {
    return Closest(_geod.Line(latX, lonX, aziX, LineCaps),
                   _geod.Line(latY, lonY, aziY, LineCaps),
                   p0, c);
  }

  Intersect::Point
  Intersect::Closest(const GeodesicLine& lineX, const GeodesicLine& lineY,
                     const Intersect::Point& p0, int* c) const {
    XPoint p = ClosestInt(lineX, lineY, XPoint(p0));
    if (c) *c = p.c;
    return p.data();
  }

  Intersect::Point
  Intersect::Segment(Math::real latX1, Math::real lonX1,
                     Math::real latX2, Math::real lonX2,
                     Math::real latY1, Math::real lonY1,
                     Math::real latY2, Math::real lonY2,
                     int& segmode, int* c) const {
    return Segment(_geod.InverseLine(latX1, lonX1, latX2, lonX2, LineCaps),
                   _geod.InverseLine(latY1, lonY1, latY2, lonY2, LineCaps),
                   segmode, c);
  }

  Intersect::Point
  Intersect::Segment(const GeodesicLine& lineX,
                     const GeodesicLine& lineY, int& segmode, int* c) const {
    XPoint p = SegmentInt(lineX, lineY, segmode);
    if (c) *c = p.c;
    return p.data();
  }

  Intersect::Point
  Intersect::Next(Math::real latX, Math::real lonX,
                  Math::real aziX, Math::real aziY, int* c) const {
    return Next(_geod.Line(latX, lonX, aziX, LineCaps),
                _geod.Line(latX, lonX, aziY, LineCaps), c);
  }

  Intersect::Point
  Intersect::Next(const GeodesicLine& lineX, const GeodesicLine& lineY,
                  int* c) const {
    XPoint p = NextInt(lineX, lineY);
    if (c) *c = p.c;
    return p.data();
  }

  std::vector<Intersect::Point>
  Intersect::All(Math::real latX, Math::real lonX, Math::real aziX,
                 Math::real latY, Math::real lonY, Math::real aziY,
                 Math::real maxdist, const Point& p0) const {
    return All(_geod.Line(latX, lonX, aziX, LineCaps),
               _geod.Line(latY, lonY, aziY, LineCaps),
               maxdist, p0);
  }

  std::vector<Intersect::Point>
  Intersect::All(Math::real latX, Math::real lonX, Math::real aziX,
                 Math::real latY, Math::real lonY, Math::real aziY,
                 Math::real maxdist, std::vector<int>& c, const Point& p0)
    const {
    return All(_geod.Line(latX, lonX, aziX, LineCaps),
               _geod.Line(latY, lonY, aziY, LineCaps),
               maxdist, c, p0);
  }

  std::vector<Intersect::Point>
  Intersect::All(const GeodesicLine& lineX, const GeodesicLine& lineY,
                 Math::real maxdist, const Point& p0) const {
    vector<int> c;
    return AllInternal(lineX, lineY, maxdist, p0, c, false);
  }

  std::vector<Intersect::Point>
  Intersect::All(const GeodesicLine& lineX, const GeodesicLine& lineY,
                 Math::real maxdist, std::vector<int>& c, const Point& p0)
    const {
    return AllInternal(lineX, lineY, maxdist, p0, c, true);
  }

  Intersect::XPoint
  Intersect::Spherical(const GeodesicLine& lineX, const GeodesicLine& lineY,
                       const Intersect::XPoint& p) const {
    // threshold for coincident geodesics and intersections; this corresponds
    // to about 4.3 nm on WGS84.
    real latX, lonX, aziX, latY, lonY, aziY;
    lineX.Position(p.x , latX, lonX, aziX);
    lineY.Position(p.y, latY, lonY, aziY);
    real z, aziXa, aziYa;
    _geod.Inverse(latX, lonX, latY, lonY, z, aziXa, aziYa);
    real sinz = sin(z/_rR), cosz = cos(z/_rR);
    // X = interior angle at X, Y = exterior angle at Y
    real dX, dY, dXY,
      X = Math::AngDiff(aziX, aziXa, dX), Y = Math::AngDiff(aziY, aziYa, dY),
      XY = Math::AngDiff(X, Y, dXY);
    real s = copysign(real(1), XY + (dXY + dY - dX)); // inverted triangle
    // For z small, sinz -> z, cosz -> 1
    // ( sinY*cosX*cosz - cosY*sinX) =
    // (-sinX*cosY*cosz + cosX*sinY) -> sin(Y-X)
    // for z = pi, sinz -> 0, cosz -> -1
    // ( sinY*cosX*cosz - cosY*sinX) -> -sin(Y+X)
    // (-sinX*cosY*cosz + cosX*sinY) ->  sin(Y+X)
    real sinX, cosX; Math::sincosde(s*X, s*dX, sinX, cosX);
    real sinY, cosY; Math::sincosde(s*Y, s*dY, sinY, cosY);
    real sX, sY;
    int c;
    if (z <= _eps * _rR) {
      sX = sY = 0;              // Already at intersection
      // Determine whether lineX and lineY are parallel or antiparallel
      if (fabs(sinX - sinY) <= _eps && fabs(cosX - cosY) <= _eps)
        c = 1;
      else if (fabs(sinX + sinY) <= _eps && fabs(cosX + cosY) <= _eps)
        c = -1;
      else
        c = 0;
    } else if (fabs(sinX) <= _eps && fabs(sinY) <= _eps) {
      c = cosX * cosY > 0 ? 1 : -1;
      // Coincident geodesics, place intersection at midpoint
      sX =  cosX * z/2; sY = -cosY * z/2;
      // alt1: sX =  cosX * z; sY = 0;
      // alt2: sY = -cosY * z; sX = 0;
    } else {
      // General case.  [SKIP: Divide args by |sinz| to avoid possible
      // underflow in {sinX,sinY}*sinz; this is probably not necessary].
      // Definitely need to treat sinz < 0 (z > pi*R) correctly.  Without
      // this we have some convergence failures in Basic.
      sX = _rR * atan2(sinY * sinz,  sinY * cosX * cosz - cosY * sinX);
      sY = _rR * atan2(sinX * sinz, -sinX * cosY * cosz + cosX * sinY);
      c = 0;
    }
    return XPoint(sX, sY, c);
  }

  Intersect::XPoint
  Intersect::Basic(const GeodesicLine& lineX, const GeodesicLine& lineY,
                   const Intersect::XPoint& p0) const {
    ++_cnt1;
    XPoint q = p0;
    for (int n = 0;
         n < numit_ ||
           GEOGRAPHICLIB_PANIC("Convergence failure in Intersect");
         ++n) {
      ++_cnt0;
      XPoint dq = Spherical(lineX, lineY, q);
      q += dq;
      if (q.c || !(dq.Dist() > _tol)) break; // break if nan
    }
    return q;
  }

  Intersect::XPoint
  Intersect::ClosestInt(const GeodesicLine& lineX, const GeodesicLine& lineY,
                        const Intersect::XPoint& p0) const {
    const int num = 5;
    const int ix[num] = { 0,  1, -1,  0,  0 };
    const int iy[num] = { 0,  0,  0,  1, -1 };
    bool    skip[num] = { 0,  0,  0,  0,  0 };
    XPoint q;                    // Best intersection so far
    for (int n = 0; n < num; ++n) {
      if (skip[n]) continue;
      XPoint qx = Basic(lineX, lineY, p0 + XPoint(ix[n] * _d1, iy[n] * _d1));
      qx = fixcoincident(p0, qx);
      if (_comp.eq(q, qx)) continue;
      if (qx.Dist(p0) < _t1) { q = qx; ++_cnt2; break; }
      if (n == 0 || qx.Dist(p0) < q.Dist(p0)) { q = qx; ++_cnt2; }
      for (int m = n + 1; m < num; ++m)
        skip[m] = skip[m] ||
          qx.Dist(p0 + XPoint(ix[m]*_d1, iy[m]*_d1)) < 2*_t1 - _d1 - _delta;
    }
    return q;
  }

  Intersect::XPoint
  Intersect::NextInt(const GeodesicLine& lineX, const GeodesicLine& lineY)
    const {
    const int num = 8;
    const int ix[num] = { -1, -1,  1,  1, -2,  0,  2,  0 };
    const int iy[num] = { -1,  1, -1,  1,  0,  2,  0, -2 };
    bool    skip[num] = {  0,  0,  0,  0,  0,  0,  0,  0 };
    XPoint z(0,0),              // for excluding the origin
      q(Math::infinity(), 0);   // Best intersection so far
    for (int n = 0; n < num; ++n) {
      if (skip[n]) continue;
      XPoint qx = Basic(lineX, lineY, XPoint(ix[n] * _d2, iy[n] * _d2));
      qx = fixcoincident(z, qx);
      bool zerop = _comp.eq(z, qx);
      if (qx.c == 0 && zerop) continue;
      if (qx.c && zerop) {
        for (int sgn = -1; sgn <= 1; sgn+=2) {
          real s = ConjugateDist(lineX, sgn * _d, false);
          XPoint qa(s, qx.c*s, qx.c);
          if (qa.Dist() < q.Dist()) { q = qa; ++_cnt2; }
        }
      } else {
        if (qx.Dist() < q.Dist()) { q = qx; ++_cnt2; }
      }
      for (int sgn = -1; sgn <= 1; ++sgn) {
        // if qx.c == 0 only process sgn == 0
        // if zerop skip sgn == 0
        if ((qx.c == 0 && sgn != 0) || (zerop && sgn == 0)) continue;
        XPoint qy = qx.c ? qx + Point(sgn * _d2, qx.c * sgn *_d2) : qx;
        for (int m = n + 1; m < num; ++m)
          skip[m] = skip[m] ||
            qy.Dist(XPoint(ix[m]*_d2, iy[m]*_d2)) < 2*_t1 - _d2 - _delta;
      }
    }
    return q;
  }

  Intersect::XPoint
  Intersect::SegmentInt(const GeodesicLine& lineX, const GeodesicLine& lineY,
                        int& segmode) const {
    // The conjecture is that whenever two geodesic segments intersect, the
    // intersection is the one that is closest to the midpoints of segments.
    // If this is proven, set conjectureproved to true.
    const bool conjectureproved = false;
    real sx = lineX.Distance(), sy = lineY.Distance();
    // p0 is center of [sx,sy] rectangle, q is intersection closest to p0
    XPoint p0 = XPoint(sx/2, sy/2), q = ClosestInt(lineX, lineY, p0);
    q = fixsegment(sx, sy, q);
    segmode = segmentmode(sx, sy, q);
    // Are corners of [sx,sy] rectangle further from p0 than q?
    if (!conjectureproved && segmode != 0 && p0.Dist() >= p0.Dist(q)) {
      int segmodex = 1;
      XPoint qx;
      // Cycle through 4 corners of [sx,sy] rectangle
      for (int ix = 0; ix < 2 && segmodex != 0; ++ix) {
        for (int iy = 0; iy < 2 && segmodex != 0; ++iy) {
          XPoint t(ix * sx, iy * sy); // corner point
          // Is corner outside next intersection exclusion circle?
          if (q.Dist(t) >= 2 * _t1) {
            ++_cnt3;
            qx = Basic(lineX, lineY, t);
            // fixsegment is not needed because the coincidence line must just
            // slice off a corner of the sx x sy rectangle.
            qx = fixcoincident(t, qx);
            // No need to check if equal to q, because result is only accepted
            // if segmode != 0 && segmodex == 0.
            segmodex = segmentmode(sx, sy, qx);
          }
        }
      }
      if (segmodex == 0) { ++_cnt4; segmode = 0; q = qx; }
    }
    return q;
  }

  std::vector<Intersect::XPoint>
  Intersect::AllInt0(const GeodesicLine& lineX,
                     const GeodesicLine& lineY,
                     Math::real maxdist, const XPoint& p0) const {
    real maxdistx = maxdist + _delta;
    const int m = int(ceil(maxdistx / _d3)), // process m x m set of tiles
      m2 = m*m + (m - 1) % 2,                // add center tile if m is even
      n = m - 1;                             // Range of i, j = [-n:2:n]
    real d3 = maxdistx/m;                    // d3 <= _d3
    vector<XPoint> start(m2);
    vector<bool> skip(m2, false);
    int h = 0, c0 = 0;
    start[h++] = p0;
    for (int i = -n; i <= n; i += 2)
      for (int j = -n; j <= n; j += 2) {
        if (!(i == 0 && j == 0))
          start[h++] = p0 + XPoint( d3 * (i + j) / 2, d3 * (i - j) / 2);
      }
    // assert(h == m2);
    set<XPoint, SetComp> r(_comp); // Intersections found
    set<XPoint, SetComp> c(_comp); // Closest coincident intersections
    vector<XPoint> added;
    for (int k = 0; k < m2; ++k) {
      if (skip[k]) continue;
      XPoint q = Basic(lineX, lineY, start[k]);
      if (r.find(q) != r.end()  // intersection already found
          // or it's on a line of coincident intersections already processed
          || (c0 != 0 && c.find(fixcoincident(p0, q)) != c.end()))
        continue;
      added.clear();
      if (q.c != 0) {
        // This value of q.c must be constitent with c0
        // assert(c0 == 0 || c0 == q.c);
        c0 = q.c;
        // Process coincident intersections
        q = fixcoincident(p0, q);
        c.insert(q);
        // Elimate all existing intersections on this line (which
        // didn't set c0).
        for (auto qp = r.begin(); qp != r.end(); ) {
          if (_comp.eq(fixcoincident(p0, *qp, c0), q)) {
            qp = r.erase(qp);
          }
          else
            ++qp;
        }
        real s0 = q.x;
        XPoint qc;
        real t, m12, M12, M21;
        lineX.GenPosition(false, s0,
                          GeodesicLine::REDUCEDLENGTH |
                          GeodesicLine::GEODESICSCALE,
                          t, t, t, t, m12, M12, M21, t);
        // Compute line of conjugate points
        for (int sgn = -1; sgn <= 1; sgn += 2) {
          real sa = 0;
          do {
            sa = ConjugateDist(lineX, s0 + sa + sgn*_d, false, m12, M12, M21)
              - s0;
            qc = q + XPoint(sa, c0*sa);
            added.push_back(qc);
            r.insert(qc);
          } while (qc.Dist(p0) <= maxdistx);
        }
      }
      added.push_back(q);
      r.insert(q);
      for (auto qp = added.cbegin(); qp != added.cend(); ++qp) {
        for (int l = k + 1; l < m2; ++l)
          skip[l] = skip[l] || qp->Dist(start[l]) < 2*_t1 - d3 - _delta;
      }
    }
    // Trim intersections to maxdist
    for (auto qp = r.begin(); qp != r.end(); ) {
      if (!(qp->Dist(p0) <= maxdist))
        qp = r.erase(qp);
      else
        ++qp;
    }
    vector<XPoint> v(r.size());
    int i = 0;
    for (auto p = r.cbegin(); p != r.cend(); ++p)
      v[i++] = *p;
    sort(v.begin(), v.end(), RankPoint(p0));
    return v;
  }

  std::vector<Intersect::Point>
  Intersect::AllInternal(const GeodesicLine& lineX, const GeodesicLine& lineY,
                         Math::real maxdist, const Point& p0,
                         std::vector<int>& c, bool cp) const {
    const vector<XPoint>
      v = AllInt0(lineX, lineY, fmax(real(0), maxdist), XPoint(p0));
    int i = int(v.size());
    vector<Point> u(i);
    if (cp) c.resize(i);
    for (int j = 0; j < i; ++j) {
      u[j] = v[j].data();
      if (cp) c[j] = v[j].c;
    }
    return u;
  }

  Math::real Intersect::distpolar(Math::real lat1, Math::real* lat2)
    const {
    GeodesicLine line = _geod.Line(lat1, 0, 0,
                                   GeodesicLine::REDUCEDLENGTH |
                                   GeodesicLine::GEODESICSCALE |
                                   GeodesicLine::DISTANCE_IN);
    real s = ConjugateDist(line, (1 + _f/2) * _a * Math::pi() / 2, true);
    if (lat2) {
      real t;
      line.GenPosition(false, s, GeodesicLine::LATITUDE,
                       *lat2, t, t, t, t, t, t, t);
    }
    return s;
  }

  Math::real Intersect::polarb(Math::real* lata, Math::real* latb) const {
    if (_f == 0) {
      if (lata) *lata = 64;
      if (latb) *latb = 90-64;
      return _d;
    }
    real
      lat0 = 63, s0 = distpolar(lat0),
      lat1 = 65, s1 = distpolar(lat1),
      lat2 = 64, s2 = distpolar(lat2),
      latx = lat2, sx = s2;
    // Solve for ds(lat)/dlat = 0 with a quadratic fit
    for (int i = 0; i < 10; ++i) {
      real den = (lat1-lat0)*s2 + (lat0-lat2)*s1 + (lat2-lat1)*s0;
      if (!(den < 0 || den > 0)) break; // Break if nan
      real latn = ((lat1-lat0)*(lat1+lat0)*s2 + (lat0-lat2)*(lat0+lat2)*s1 +
                   (lat2-lat1)*(lat2+lat1)*s0) / (2*den);
      lat0 = lat1; s0 = s1;
      lat1 = lat2; s1 = s2;
      lat2 = latn; s2 = distpolar(lat2);
      if (_f < 0 ? (s2 < sx) : (s2 > sx)) {
        sx = s2;
        latx = lat2;
      }
    }
    if (lata) *lata = latx;
    if (latb) distpolar(latx, latb);
    return 2 * sx;
  }

  // Find {semi-,}conjugate point relative to s0 which is close to s1.
  Math::real Intersect::ConjugateDist(const GeodesicLine& line, Math::real s3,
                                      bool semi, Math::real m12,
                                      Math::real M12, Math::real M21) const {
    // semi = false: solve for m23 = 0 using dm23/ds3 = M32
    // semi = true : solve for M23 = 0 using dM23/ds3 = - (1 - M23*M32)/m23
    // Here 2 is point with given m12, M12, M21 and default values s.t. point 2
    // = point 1.
    real s = s3;
    for (int i = 0; i < 100; ++i) {
      real t, m13, M13, M31;
      line.GenPosition(false, s,
                       GeodesicLine::REDUCEDLENGTH |
                       GeodesicLine::GEODESICSCALE,
                       t, t, t, t, m13, M13, M31, t);
      real
        // See "Algorithms for geodesics", eqs. 31, 32, 33.
        m23 = m13 * M12 - m12 * M13,
        // when m12 -> eps, (1 - M12 * M21) -> eps^2, I suppose.
        M23 = M13 * M21 + (m12 == 0 ? 0 : (1 - M12 * M21) * m13/m12),
        M32 = M31 * M12 + (m13 == 0 ? 0 : (1 - M13 * M31) * m12/m13);
      real ds = semi ? m23 * M23 / (1 - M23*M32) : -m23 / M32;
      s = s + ds;
      if (!(fabs(ds) > _tol)) break;
    }
    return s;
  }

  Math::real Intersect::conjdist(Math::real azi,
                                 Math::real* ds,
                                 Math::real* sp, Math::real* sm) const {
    GeodesicLine line = _geod.Line(0, 0, azi, LineCaps);
    real s = ConjugateDist(line, _d, false);
    if (ds) {
      XPoint p = Basic(line, line, XPoint(s/2, -3*s/2));
      if (sp) *sp = p.x;
      if (sm) *sm = p.y;
      *ds = p.Dist() - 2*s;
    }
    return s;
  }

  Math::real Intersect::distoblique(Math::real* azi,
                                    Math::real* sp,
                                    Math::real* sm) const {
    if (_f == 0) {
      if (azi) *azi = 45;
      if (sp) *sp = 0.5;
      if (sm) *sm = -1.5;
      return _d;
    }
    real sa, sb,
      azi0 = 46, ds0, s0 = conjdist(azi0, &ds0, &sa, &sb),
      azi1 = 44, ds1, s1 = conjdist(azi1, &ds1, &sa, &sb),
      azix = azi1, dsx = fabs(ds1), sx = s1, sax = sa, sbx = sb;
    // find ds(azi) = 0 by secant method
    (void) s0;
    for (int i = 0; i < 10 && ds1 != ds0; ++i) {
      real azin = (azi0*ds1-azi1*ds0)/(ds1-ds0);
      azi0 = azi1; s0 = s1; ds0 = ds1;
      azi1 = azin; s1 = conjdist(azi1, &ds1, &sa, &sb);
      if (fabs(ds1) < dsx) {
        azix = azi1, sx = s1, dsx = fabs(ds1);
        sax = sa; sbx = sb;
        if (ds1 == 0) break;
      }
    }
    if (azi) *azi = azix;
    if (sp) *sp = sax;
    if (sm) *sm = sbx;
    return sx;
  }

  Intersect::XPoint
  Intersect::fixcoincident(const Intersect::XPoint& p0,
                           const Intersect::XPoint& p) {
    return fixcoincident(p0, p, p.c);
  }

  Intersect::XPoint
  Intersect::fixcoincident(const Intersect::XPoint& p0,
                           const Intersect::XPoint& p, int c) {
    if (c == 0) return p;
    // eqs : [p0x-p1x = -c*(p0y-p1y), p1x = px+s, p1y = py+c*s]$
    // sol : solve(eqs,[s,p1x,p1y]);
    // =>
    // sol:[ s = ((p0x+c*p0y) - (px+c*py))/2,
    //       p1x = px +     ((p0x+c*p0y) - (px+c*py))/2,
    //       p1y = py + c * ((p0x+c*p0y) - (px+c*py))/2
    // ];
    real s = ((p0.x + c * p0.y) - (p.x + c * p.y))/2;
    return p + XPoint(s, c*s);
  }

  Intersect::XPoint
  Intersect::fixsegment(Math::real sx, Math::real sy,
                        const Intersect::XPoint& p) {
    if (p.c == 0) return p;
    // eq0: [p1x = px+s, p1y = py+f*s]$
    // solx0:linsolve(cons(p1x=0 ,eq0),[s,p1x,p1y]);
    // solx1:linsolve(cons(p1x=sx,eq0),[s,p1x,p1y]);
    // soly0:linsolve(cons(p1y=0 ,eq0),[s,p1x,p1y]);
    // soly1:linsolve(cons(p1y=sy,eq0),[s,p1x,p1y]);
    // solx0:[s = -px      ,p1x = 0 ,p1y = py-f*px     ];
    // solx1:[s = sx-px    ,p1x = sx,p1y = py-f*(px-sx)];
    // soly0:[s = -f*py    ,p1x = px-f*py     ,p1y = 0 ];
    // soly1:[s = f*(sy-py),p1x = px-f*(py-sy),p1y = sy];
    real
      pya = p.y - p.c *  p.x,     sa =             -p.x,  // pxa = 0
      pyb = p.y - p.c * (p.x-sx), sb =         sx - p.x,  // pxb = sx
      pxc = p.x - p.c *  p.y,     sc = p.c *      -p.y,  // pyc = 0
      pxd = p.x - p.c * (p.y-sy), sd = p.c * (sy - p.y); // pyd = sy
    bool
      ga = 0 <= pya && pya <= sy,
      gb = 0 <= pyb && pyb <= sy,
      gc = 0 <= pxc && pxc <= sx,
      gd = 0 <= pxd && pxd <= sx;
    real s;
    // Test opposite sides of the rectangle first
    if      (ga && gb) s = (sa + sb) / 2;
    else if (gc && gd) s = (sc + sd) / 2;
    else if (ga && gc) s = (sa + sc) / 2;
    else if (ga && gd) s = (sa + sd) / 2;
    else if (gb && gc) s = (sb + sc) / 2;
    else if (gb && gd) s = (sb + sd) / 2;
    else {
      // Intersection not within segments; place intersection in smallest gap.
      if (p.c > 0) {
        // distance from p to corner p0 is abs( (px - py) - (p0x - p0y) )
        // consider corners p0 = [0, sy] and p0 = [sx, 0]
        if (fabs((p.x - p.y) + sy) < fabs((p.x - p.y) - sx))
          s = (sy - (p.x + p.y))/2;
        else
          s = (sx - (p.x + p.y))/2;
      } else {
        // distance from p to corner p0 is abs( (px + p.y) - (p0x + p0y) )
        // consider corners p0 = [0, 0] and p0 = [sx, sy]
        if (fabs(p.x + p.y) < fabs((p.x + p.y) - (sx + sy)))
          s = (0 - (p.x - p.y))/2;
        else
          s = ((sx - sy) - (p.x - p.y))/2;
      }
    }
    return p + XPoint(s, p.c*s);
  }

}

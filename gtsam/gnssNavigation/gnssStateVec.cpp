/** @file   gnssStateVec.cpp
 * @brief  gnss state vector -- {x,y,z,cb,tz}
 * @author Ryan Watson
 */

#include <gtsam/gnssNavigation/gnssStateVec.h>
#include <cmath>

using namespace std;

namespace gtsam {

#ifndef GTSAM_TYPEDEF_POINTS_TO_VECTORS
bool gnssStateVec::equals(const gnssStateVec &q, double tol) const {
        return (fabs(x() - q.x()) < tol && fabs(y() - q.y()) < tol &&
                fabs(z() - q.z())<tol && fabs(cb() - q.cb())< tol &&
                fabs(tz() - q.tz())<tol);
}

void gnssStateVec::print(const string& s) const {
        cout << s << *this << endl;
}

/* ************************************************************************* */
double gnssStateVec::distance(const gnssStateVec &q, OptionalJacobian<1, 5> H1,
                              OptionalJacobian<1, 5> H2) const {
        return gtsam::distance5(*this,q,H1,H2);
}

double gnssStateVec::norm(OptionalJacobian<1,5> H) const {
        return gtsam::norm5(*this, H);
}

double gnssStateVec::dot(const gnssStateVec &q, OptionalJacobian<1, 5> H1,
                         OptionalJacobian<1, 5> H2) const {
        return gtsam::dot(*this, q, H1, H2);
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const gnssStateVec& p) {
        os << "   " << '[' << p.x() << ", " << p.y() << ", " << p.z() << ", " << p.cb() << ", " << p.tz() << "]'";
        return os;
}

/* ************************************************************************* */
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
gnssStateVec gnssStateVec::add(const gnssStateVec &q, OptionalJacobian<5,5> H1,
                               OptionalJacobian<5,5> H2) const {
        if (H1) *H1 = I_5x5;
        if (H2) *H2 = I_5x5;
        return *this + q;
}

gnssStateVec gnssStateVec::sub(const gnssStateVec &q, OptionalJacobian<5,5> H1,
                               OptionalJacobian<5,5> H2) const {
        if (H1) *H1 = I_5x5;
        if (H2) *H2 = -I_5x5;
        return *this - q;
}
#endif

#endif
/* ************************************************************************* */
double distance5(const gnssStateVec &p1, const gnssStateVec &q, OptionalJacobian<1, 5> H1,
                 OptionalJacobian<1, 5> H2) {
        double range = (q - p1).norm();
        if (H1) {
                *H1 << p1.x() - q.x(), p1.y() - q.y(), p1.z() - q.z(), p1.cb() - q.cb(), p1.tz() - q.tz();
                *H1 = *H1 *(1. / range);
        }
        if (H2) {
                *H2 << -p1.x() + q.x(), -p1.y() + q.y(), -p1.z() + q.z(), -q.cb() + q.cb(), -q.tz() + q.tz();
                *H2 = *H2 *(1. / range);
        }
        return range;
}

// returns estimated pseudoragne
double norm5(const gnssStateVec &p, OptionalJacobian<1, 5> H) {
        double r = sqrt(p.x() * p.x() + p.y() * p.y() + p.z() * p.z()) + p.cb() + p.tz();
        if (H) {
                if (fabs(r) > 1e-10)
                        *H << p.x() / r, p.y() / r, p.z() / r, 1, 1;
                else
                        *H << 1, 1, 1, 1, 1;  // really infinity, why 1 ?
        }
        return r;
}

double dot(const gnssStateVec &p, const gnssStateVec &q, OptionalJacobian<1, 5> H1,
           OptionalJacobian<1, 5> H2) {
        if (H1) *H1 << q.x(), q.y(), q.z(), q.cb(), q.tz();
        if (H2) *H2 << p.x(), p.y(), p.z(), p.cb(), p.tz();
        return p.x() * q.x() + p.y() * q.y() + p.z() * q.z() + p.cb() * q.cb() + p.tz()*q.tz();
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const gtsam::Point5Pair &p) {
        os << p.first << " <-> " << p.second;
        return os;
}
/* ************************************************************************* */
} // namespace gtsam

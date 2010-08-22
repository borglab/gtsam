/**
 * @file LieVector.h
 * @brief A wrapper around vector providing Lie compatibility
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/Lie.h>

namespace gtsam {
/** temporarily using the old system */

// The rest of the file makes double and Vector behave as a Lie type (with + as compose)

// Vector group operations
inline Vector compose(const Vector& p1,const Vector& p2) { return p1+p2;}
inline Vector inverse(const Vector& p) { return -p;}
inline Vector between(const Vector& p1,const Vector& p2) { return p2-p1;}

// Vector is a trivial Lie group
template<> inline Vector expmap(const Vector& d) { return d;}
template<> inline Vector expmap(const Vector& p,const Vector& d) { return p+d;}
template<> inline Vector logmap(const Vector& p) { return p;}
template<> inline Vector logmap(const Vector& p1,const Vector& p2) { return p2-p1;}

} // \namespace gtsam

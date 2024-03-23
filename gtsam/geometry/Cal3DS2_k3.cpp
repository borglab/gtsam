/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2_k3.cpp
 * @date Dec 19, 2023
 * @author demul
 * @author Hyeonjin Jeong
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2_k3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam
{
/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3DS2_k3& cal)
{
    os << (Cal3DS2_k3_Base&)cal;
    return os;
}

/* ************************************************************************* */
void Cal3DS2_k3::print(const std::string& s_) const
{
    Base::print(s_);
}

/* ************************************************************************* */
bool Cal3DS2_k3::equals(const Cal3DS2_k3& K, double tol) const
{
    const Cal3DS2_k3_Base* base = dynamic_cast<const Cal3DS2_k3_Base*>(&K);
    return Cal3DS2_k3_Base::equals(*base, tol);
}

/* ************************************************************************* */
Cal3DS2_k3 Cal3DS2_k3::retract(const Vector& d) const
{
    return Cal3DS2_k3(vector() + d);
}

/* ************************************************************************* */
Vector Cal3DS2_k3::localCoordinates(const Cal3DS2_k3& T2) const
{
    return T2.vector() - vector();
}
}  // namespace gtsam
/* ************************************************************************* */

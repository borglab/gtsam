/**
 *  @file   PhaseFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for carrier-phase factor
 **/

#include <gtsam/gnssNavigation/PhaseFactor.h>

using namespace std;

namespace gtsam
{
//***************************************************************************
Vector PhaseFactor::evaluateError(const nonBiasStates &q, const phaseBias &g, boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
{

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        Matrix gnssPartials = Z_1x1;

        // if (H1) { (*H1) = (Matrix(1,5) << h).finished(); }
        if (H1)
        {
                H1->resize(1, 5);
                *H1 << h(0), h(1), h(2), h(3), h(4);
        }
        if (H2)
        {
                gnssPartials(0) = 1.0; // phase bias
                (*H2) = gnssPartials;
        }
        double est = (h.transpose() * q) + g[0];
        return (Vector(1) << est - measured_).finished();
}
} // namespace gtsam

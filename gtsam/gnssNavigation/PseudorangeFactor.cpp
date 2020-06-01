/**
 *  @file   PseudorangeFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for pseudorange factor
 **/

#include "PseudorangeFactor.h"

using namespace std;

namespace gtsam
{

//***************************************************************************
Vector PseudorangeFactor::evaluateError(const nonBiasStates &q,
                                        boost::optional<Matrix &> H) const
{

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        // if (H) { (*H) = (Matrix(1,5) << h ).finished(); }
        if (H)
        {
                H->resize(1, 5);
                *H << h(0), h(1), h(2), h(3), h(4);
        }
        double est = (h.transpose() * q);
        return (Vector(1) << est - measured_).finished();
}
} // namespace gtsam

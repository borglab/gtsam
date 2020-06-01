/**
 *  @file   GnssBetweenFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for GnssBetweenFactor
 **/

#include <gtsam/gnssNavigation/GnssBetweenFactor.h>

using namespace std;

namespace gtsam {
//***************************************************************************
Vector GnssBetweenFactor::evaluateError(const nonBiasStates& q, const nonBiasStates& p, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

        Vector h = (Matrix(1,5)<<1,1,1,1,1).finished();

        if (H1) { (*H1) = (Matrix(1,5) << h ).finished(); }
        if (H2) { (*H2) = (Matrix(1,5) << h ).finished(); }
        double est = norm5(q-p);
        return (Vector(1) << est ).finished();
}
}  //namespace

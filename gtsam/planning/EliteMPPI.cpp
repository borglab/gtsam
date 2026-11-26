// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file EliteMPPI.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <gtsam/planning/EliteMPPI.h>

#include <limits>

namespace gtsam {

EliteMPPI::EliteMPPI(size_t elites, double inverseTemperature, double minStddev)
    : selection_(elites, minStddev),
      weighting_(inverseTemperature),
      minStddev_(minStddev) {}

Vector EliteMPPI::weights(const Vector& costs) const {
  // Mask non-elite candidates to +infinity before Boltzmann weighting.
  const Vector elites = selection_.weights(costs);
  const Vector masked =
      (elites.array() > 0.)
          .select(costs, std::numeric_limits<double>::infinity());
  return weighting_.weights(masked);
}

}  // namespace gtsam

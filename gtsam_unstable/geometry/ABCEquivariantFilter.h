/**
 * @file ABCEquivariantFilter.h
 * @brief Forwarding header for AbcEqF.h.
 *
 * @deprecated Include <gtsam_unstable/geometry/AbcEqF.h> and use abc::AbcEqF.
 * Kept because gtsam/navigation/navigation_geometric.i and
 * examples/AbcEquivariantFilterExample.cpp still use the old names.
 */

#pragma once

#include <gtsam_unstable/geometry/AbcEqF.h>

namespace gtsam {
namespace abc {

/// @deprecated Use AbcEqF.
template <size_t N>
using AbcEquivariantFilter = AbcEqF<N>;

}  // namespace abc
}  // namespace gtsam

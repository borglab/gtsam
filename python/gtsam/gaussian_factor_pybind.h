#pragma once

/* Surface unregistered GaussianFactor subclasses as their nearest registered
 * base.
 *
 * pybind11 resolves a polymorphic return value by looking up typeid(*src). If
 * the dynamic type is not registered it does NOT walk up the hierarchy -- it
 * falls back to the *declared* return type. NonlinearFactor::linearize is
 * declared to return gtsam::GaussianFactor*, so any concrete factor type that
 * is not registered surfaces in Python as a bare GaussianFactor, losing the
 * JacobianFactor interface (getA, getb, jacobian, augmentedJacobian, ...).
 *
 * That is what happens to FixedJacobianFactor<M, Ns...>, which is templated on
 * its dimensions and so cannot practically be registered: every fixed-size
 * NoiseModelFactorT linearizes to one. gtsam::BetweenFactor names a fixed-size
 * TangentVector as its error type and is affected; gtsam::PriorFactor goes
 * through NoiseModelFactorN, whose error type is a dynamic Vector, so it keeps
 * the generic path and a plain JacobianFactor.
 *
 * The hook checks the registry first, so registered subclasses keep their own
 * identity -- gtsam::GaussianConditional derives from JacobianFactor and must
 * still arrive in Python as a GaussianConditional.
 *
 * This lives in a header included by both module templates rather than in a
 * per-module preamble: linearize is declared in nonlinear.i while the factor
 * types are registered from linear.i, and pybind11 needs the hook visible in
 * every translation unit that casts a GaussianFactor.
 *
 * Two limitations worth knowing about:
 *
 *  - The list of fallback bases below is hardcoded. C++ RTTI offers no way to
 *    enumerate base classes, so any hook of this shape has to name them. If a
 *    further branch is registered under GaussianFactor, add it here.
 *
 *  - detail::get_type_info is pybind11 internal API. It is long-stable and
 *    widely used, but it is not a public guarantee. The alternative -- naming
 *    the registered subclasses that must keep their identity, currently just
 *    GaussianConditional -- would be brittle in a different way.
 */

#include <pybind11/pybind11.h>

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

#include <typeinfo>

namespace pybind11 {

template <>
struct polymorphic_type_hook<gtsam::GaussianFactor> {
  static const void *get(const gtsam::GaussianFactor *src,
                         const std::type_info *&type) {
    if (src == nullptr) {
      type = nullptr;
      return nullptr;
    }

    // A registered dynamic type keeps its own identity.
    const std::type_info &dynamic_type = typeid(*src);
    if (detail::get_type_info(dynamic_type) != nullptr) {
      type = &dynamic_type;
      return dynamic_cast<const void *>(src);
    }

    // Otherwise expose the nearest registered base rather than letting
    // pybind11 fall back to the declared return type.
    if (const auto *jacobian = dynamic_cast<const gtsam::JacobianFactor *>(src)) {
      type = &typeid(gtsam::JacobianFactor);
      return jacobian;
    }
    if (const auto *hessian = dynamic_cast<const gtsam::HessianFactor *>(src)) {
      type = &typeid(gtsam::HessianFactor);
      return hessian;
    }

    type = &dynamic_type;
    return dynamic_cast<const void *>(src);
  }
};

}  // namespace pybind11

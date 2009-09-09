/**
 * @file    Value.h
 * @brief   Abstract base class for values that can be updated using exmap
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "Vector.h"
#include "Testable.h"

namespace gtsam {

  /**
   * The value class should be templated with the derived class, e.g.
   * class Rot3 : public Value<Rot3>. This allows us to define the
   * return type of exmap as a Rot3 as well.
   */
  template <class Derived> class Value : public Testable<Derived> {

  private:

    const size_t dim_; // dimensionality of tangent space, e.g. 3 for Rot3

  public:

    Value(size_t dim) : dim_ (dim) {}

    /**
     * dimensionality of tangent space, e.g. 3 for Rot3
     */
    size_t dim() { return dim_;} 

    /**
     * Exponential map: add a delta vector, addition for most simple
     * types, but fully exponential map for types such as Rot3, which
     * takes a 3-dim delta vector to update a 9-dim representation.
     * equality up to tolerance
     */
    virtual Derived exmap(const Vector& delta) const = 0;
  };
}

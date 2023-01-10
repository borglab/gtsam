/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    simulated2DConstraints.h
 * @brief   measurement functions and constraint definitions for simulated 2D robot
 * @author  Alex Cunningham
 */

// \callgraph

#pragma once

#include <gtsam/base/numericalDerivative.h>

#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BoundingConstraint.h>
#include <tests/simulated2D.h>
#include "gtsam/nonlinear/NonlinearFactor.h"

// \namespace

namespace simulated2D {

  namespace equality_constraints {

    /** Typedefs for regular use */
    typedef NonlinearEquality1<Point2> UnaryEqualityConstraint;
    typedef NonlinearEquality1<Point2> UnaryEqualityPointConstraint;
    typedef BetweenConstraint<Point2> OdoEqualityConstraint;

    /** Equality between variables */
    typedef NonlinearEquality2<Point2> PoseEqualityConstraint;
    typedef NonlinearEquality2<Point2> PointEqualityConstraint;

  } // \namespace equality_constraints

  namespace inequality_constraints {

    /**
     * Unary inequality constraint forcing a coordinate to be greater/less than a fixed value (c)
     * @tparam VALUE is the value type for the variable constrained, e.g. Pose2, Point3, etc.
     * @tparam IDX is an index in tangent space to constrain, must be less than KEY::VALUE::Dim()
     */
    template<class VALUE, unsigned int IDX>
    struct ScalarCoordConstraint1: public BoundingConstraint1<VALUE> {
      typedef BoundingConstraint1<VALUE> Base;  ///< Base class convenience typedef
      typedef ScalarCoordConstraint1<VALUE, IDX> This; ///< This class convenience typedef
      typedef boost::shared_ptr<ScalarCoordConstraint1<VALUE, IDX> > shared_ptr; ///< boost::shared_ptr convenience typedef
      typedef VALUE Point; ///< Constrained variable type

      ~ScalarCoordConstraint1() override {}

      /// @return a deep copy of this factor
      gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

      /**
       * Constructor for constraint
       * @param key is the label for the constrained variable
       * @param c is the measured value for the fixed coordinate
       * @param isGreaterThan is a flag to set inequality as greater than or less than
       * @param mu is the penalty function gain
       */
      ScalarCoordConstraint1(Key key, double c,
          bool isGreaterThan, double mu = 1000.0) :
            Base(key, c, isGreaterThan, mu) {
      }

      /**
       * Access function for the constrained index
       * @return the index for the constrained coordinate
       */
      inline unsigned int index() const { return IDX; }

      /**
       * extracts a single value from the point to compute error
       * @param x is the estimate of the constrained variable being evaluated
       * @param H is an optional Jacobian, linearized at x
       */
      double value(const Point& x, OptionalMatrixType H =
          OptionalNone) const override {
        if (H) {
          Matrix D = Matrix::Zero(1, traits<Point>::GetDimension(x));
          D(0, IDX) = 1.0;
          *H = D;
        }
        return traits<Point>::Logmap(x)(IDX);
      }
    };

    /** typedefs for use with simulated2D systems */
    typedef ScalarCoordConstraint1<Point2, 0> PoseXInequality; ///< Simulated2D domain example factor constraining X
    typedef ScalarCoordConstraint1<Point2, 1> PoseYInequality; ///< Simulated2D domain example factor constraining Y

    /**
     * Trait for distance constraints to provide distance
     * @tparam T1 is a Lie value for which distance functions exist
     * @tparam T2 is a Lie value for which distance functions exist
     * @param a is the first Lie element
     * @param b is the second Lie element
     * @return a scalar distance between values
     */
    template<class T1, class T2>
    double range_trait(const T1& a, const T2& b) { return distance2(a, b); }

    /**
     * Binary inequality constraint forcing the range between points
     * to be less than or equal to a bound
     * @tparam VALUES is the variable set for the graph
     * @tparam KEY is the type of the keys for the variables constrained
     */
    template<class VALUE>
    struct MaxDistanceConstraint : public BoundingConstraint2<VALUE, VALUE> {
      typedef BoundingConstraint2<VALUE, VALUE> Base;  ///< Base class for factor
      typedef MaxDistanceConstraint<VALUE> This;  ///< This class for factor
      typedef VALUE Point; ///< Type of variable constrained

      ~MaxDistanceConstraint() override {}

      /// @return a deep copy of this factor
      gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

      /**
       * Primary constructor for factor
       * @param key1 is the first variable key
       * @param key2 is the second variable key
       * @param range_bound is the maximum range allowed between the variables
       * @param mu is the gain for the penalty function
       */
      MaxDistanceConstraint(Key key1, Key key2, double range_bound, double mu = 1000.0) :
        Base(key1, key2, range_bound, false, mu) {}

      /**
       * computes the range with derivatives
       * @param x1 is the first variable value
       * @param x2 is the second variable value
       * @param H1 is an optional Jacobian in x1
       * @param H2 is an optional Jacobian in x2
       * @return the distance between the variables
       */
      double value(const Point& x1, const Point& x2,
          OptionalMatrixType H1 = OptionalNone,
          OptionalMatrixType H2 = OptionalNone) const override {
        if (H1) *H1 = numericalDerivative21(range_trait<Point,Point>, x1, x2, 1e-5);
        if (H1) *H2 = numericalDerivative22(range_trait<Point,Point>, x1, x2, 1e-5);
        return range_trait(x1, x2);
      }
    };

    typedef MaxDistanceConstraint<Point2> PoseMaxDistConstraint; ///< Simulated2D domain example factor

    /**
     * Binary inequality constraint forcing a minimum range
     * NOTE: this is not a convex function!  Be careful with initialization.
     * @tparam POSE is the type of the pose value constrained
     * @tparam POINT is the type of the point value constrained
     */
    template<class POSE, class POINT>
    struct MinDistanceConstraint : public BoundingConstraint2<POSE, POINT> {
      typedef BoundingConstraint2<POSE, POINT> Base; ///< Base class for factor
      typedef MinDistanceConstraint<POSE, POINT> This; ///< This class for factor
      typedef POSE Pose; ///< Type of pose variable constrained
      typedef POINT Point; ///< Type of point variable constrained

      ~MinDistanceConstraint() override {}

      /// @return a deep copy of this factor
      gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

      /**
       * Primary constructor for factor
       * @param key1 is the first variable key
       * @param key2 is the second variable key
       * @param range_bound is the minimum range allowed between the variables
       * @param mu is the gain for the penalty function
       */
      MinDistanceConstraint(Key key1, Key key2,
          double range_bound, double mu = 1000.0)
      : Base(key1, key2, range_bound, true, mu) {}

      /**
       * computes the range with derivatives
       * @param x1 is the first variable value
       * @param x2 is the second variable value
       * @param H1 is an optional Jacobian in x1
       * @param H2 is an optional Jacobian in x2
       * @return the distance between the variables
       */
      double value(const Pose& x1, const Point& x2,
          OptionalMatrixType H1 = OptionalNone,
          OptionalMatrixType H2 = OptionalNone) const override {
        if (H1) *H1 = numericalDerivative21(range_trait<Pose,Point>, x1, x2, 1e-5);
        if (H1) *H2 = numericalDerivative22(range_trait<Pose,Point>, x1, x2, 1e-5);
        return range_trait(x1, x2);
      }
    };

    typedef MinDistanceConstraint<Point2, Point2> LandmarkAvoid; ///< Simulated2D domain example factor


  } // \namespace inequality_constraints

} // \namespace simulated2D


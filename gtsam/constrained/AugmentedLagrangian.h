/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AugmentedLagrangian.h
 * @brief   Augmented Lagrangian function implemented as a nonlinear factor
 * graph.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constrained/ConstrainedOptProblem.h>

namespace gtsam {

/**
 * Lagrange dual function for equality constraints and inequality constraints
 *   m(x) = 0.5 * ||f(x)||^2 - lambdaEq * h(x) + 0.5 * muEq * ||h(x)||^2
 *                           - lambdaIneq * g(x) + 0.5 * muIneq * ||g(x)_-||^2
 * To express in nonlinear least squares form, it is rewritten as
 *     m(x)
 *   = m(x) + 0.5 * epsilon * ||g(x)||^2
 *   = 0.5 * ||f(x)||^2
 *     + (0.5 * muEq * ||h(x)||^2 - lambdaEq * h(x))
 *     + (0.5 * epsilon * ||g(x)||^2 - lambdaIneq * g(x))
 *     + 0.5 * muIneq * ||g(x)_-||^2
 *     - 0.5 * epsilon * ||g(x)||^2
 *   = 0.5 * ||f(x)||^2
 *     + 0.5 * muEq * ||h(x)- lambdaEq/muEq||^2
 *     + 0.5 * epsilon * ||g(x)-lambdaIneq/muIneq||^2
 *     + 0.5 * muIneq * ||g(x)_-||^2
 *     - 0.5 * epsilon * ||g(x)||^2
 *     - c
 * where
 *   c = ||lambdaEq||^2 / (2 * muEq) + ||lambdaIneq||^2 / (2 * epsilon)
 * is a constant term,
 * and epsilon can be any positive scalar value.
 *
 * Notice: the purpose of epsilon is to incorporate (-lambdaIneq * g(x)) in
 * nonlinear least squares form. To do so, we manually create an additional
 * term (0.5 * epsilon * ||g(x)||^2), which is added and then subtracted in
 * the merit function. The term (-lambdaIneq * g(x)) and (0.5 * epsilon *
 * ||g(x)||^2) can be combined as a least-square term, and the subtraction of
 * (0.5 * epsilon * ||g(x)||^2) can be performed with anti-factor.
 * @return: factor graph representing m(x) + 0.5d * ||g(x)||^2 + c
 */
NonlinearFactorGraph AugmentedLagrangianFunction(
    const ConstrainedOptProblem& problem, const std::vector<Vector>& lambdaEq,
    const std::vector<double>& lambdaIneq, double muEq, double muIneq,
    InequalityPenaltyFunction::shared_ptr ineqConstraintPenaltyFunction,
    const double epsilon = 1.0);
}  // namespace gtsam

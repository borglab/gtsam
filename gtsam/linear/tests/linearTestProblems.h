/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    linearTestProblems.h
 * @brief   Small test problems for linear unit tests
 * @author  Richard Roberts
 */

#include <boost/assign/list_of.hpp>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Ordering.h>

using boost::assign::list_of;
using namespace gtsam;

static const Key x1=1, x2=2, x3=3, x4=4, x5=5, l1=11, l2=12, l3=13, l4=14;

static const SharedDiagonal chainNoise = noiseModel::Isotropic::Sigma(1, 0.5);
static const GaussianFactorGraph chain = list_of
  (JacobianFactor(x2, (Matrix(1, 1) << 1.), x1, (Matrix(1, 1) << 1.), (Vector(1) << 1.),  chainNoise))
  (JacobianFactor(x2, (Matrix(1, 1) << 1.), x3, (Matrix(1, 1) << 1.), (Vector(1) << 1.),  chainNoise))
  (JacobianFactor(x3, (Matrix(1, 1) << 1.), x4, (Matrix(1, 1) << 1.), (Vector(1) << 1.),  chainNoise))
  (JacobianFactor(x4, (Matrix(1, 1) << 1.), (Vector(1) << 1.),  chainNoise));
static const Ordering chainOrdering = Ordering(list_of(x2)(x1)(x3)(x4));

// Simple graph
//  l1
//  | \
// x1--x2
static const SharedDiagonal unit2 = noiseModel::Unit::Create(2);
static const GaussianFactorGraph simpleGraph = list_of
    // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
    (JacobianFactor(x1, 10*eye(2), -1.0*ones(2), unit2))
    // odometry between x1 and x2: x2-x1=[0.2;-0.1]
    (JacobianFactor(x1, -10*eye(2), x2, 10*eye(2), (Vector(2) << 2.0, -1.0), unit2))
    // measurement between x1 and l1: l1-x1=[0.0;0.2]
    (JacobianFactor(x1, -5*eye(2), l1, 5*eye(2), (Vector(2) << 0.0, 1.0), unit2))
    // measurement between x2 and l1: l1-x2=[-0.2;0.3]
    (JacobianFactor(x2, -5*eye(2), l1, 5*eye(2), (Vector(2) << -1.0, 1.5), unit2));

// Simple graph 2
//  l1   l2   l3  l4
//  | \    \ / | / |
// x1--x2--x3--x4--x5
static const SharedDiagonal noise2 = noiseModel::Isotropic::Sigma(2, 0.5);
static const GaussianFactorGraph simpleGraph2 = list_of
    // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
    (JacobianFactor(x1, 10*eye(2), -1.0*ones(2), noise2))
    // odometry between x1 and x2: x2-x1=[0.2;-0.1]
    (JacobianFactor(x1, -10*eye(2), x2, 10*eye(2), (Vector(2) << 2.0, -1.0), noise2))
    // measurement between x1 and l1: l1-x1=[0.0;0.2]
    (JacobianFactor(x1, -5*eye(2), l1, 5*eye(2), (Vector(2) << 0.0, 1.0), noise2))
    // measurement between x2 and l1: l1-x2=[-0.2;0.3]
    (JacobianFactor(x2, -5*eye(2), l1, 5*eye(2), (Vector(2) << -1.0, 1.5), noise2))

    // odometry between x2 and x3: x3-x2=[0.2;-0.1]
    (JacobianFactor(x1, -10*eye(2), x2, 10*eye(2), (Vector(2) << 2.0, -1.0), noise2))
    // measurement between x3 and l2: l2-x3=[0.0;0.2]
    (JacobianFactor(x3, -5*eye(2), l2, 5*eye(2), (Vector(2) << 0.0, 1.0), noise2))
    // measurement between x3 and l3: l3-x3=[0.0;0.2]
    (JacobianFactor(x3, -5*eye(2), l3, 5*eye(2), (Vector(2) << 0.0, 1.0), noise2))

    // odometry between x3 and x4: x4-x3=[0.2;-0.1]
    (JacobianFactor(x3, -10*eye(2), x4, 10*eye(2), (Vector(2) << 2.0, -1.0), noise2))
    // measurement between x4 and l3: l3-x3=[0.0;0.2]
    (JacobianFactor(x4, -5*eye(2), l3, 5*eye(2), (Vector(2) << 0.0, 1.0), noise2))
    // measurement between x4 and l4: l3-x3=[0.0;0.2]
    (JacobianFactor(x4, -5*eye(2), l4, 5*eye(2), (Vector(2) << 0.0, 1.0), noise2))

    // odometry between x4 and x5: x4-x3=[0.2;-0.1]
    (JacobianFactor(x4, -10*eye(2), x5, 10*eye(2), (Vector(2) << 2.0, -1.0), noise2))
    // measurement between x5 and l4: l3-x3=[0.0;0.2]
    (JacobianFactor(x5, -5*eye(2), l4, 5*eye(2), (Vector(2) << 0.0, 1.0), noise2))
    ;

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QcqpProblem.cpp
 * @brief   QCQP problem implementations.
 * @author  Frank Dellaert
 */

#include <gtsam/constrained/QcqpProblem.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>

namespace gtsam {

namespace {

template <typename T>
Values ExtractTypedQcqpValues(const Values& qcqpValues) {
  Values values;
  for (const auto& [key, value] : ExtractQcqpValues<T, 1>(qcqpValues)) {
    values.insert(key, value);
  }
  return values;
}

}  // namespace

/* ************************************************************************* */
Matrix qcqpValue(const Rot2& value) {
  return traits<Rot2>::QcqpValue<1>(value);
}

/* ************************************************************************* */
Matrix qcqpValue(const Rot3& value) {
  return traits<Rot3>::QcqpValue<1>(value);
}

/* ************************************************************************* */
Matrix qcqpValue(const Pose2& value) {
  return traits<Pose2>::QcqpValue<1>(value);
}

/* ************************************************************************* */
Matrix qcqpValue(const Pose3& value) {
  return traits<Pose3>::QcqpValue<1>(value);
}

/* ************************************************************************* */
void insertQcqpValue(Key key, const Rot2& value, Values& qcqpValues) {
  InsertQcqpValue<Rot2, 1>(key, value, &qcqpValues);
}

/* ************************************************************************* */
void insertQcqpValue(Key key, const Rot3& value, Values& qcqpValues) {
  InsertQcqpValue<Rot3, 1>(key, value, &qcqpValues);
}

/* ************************************************************************* */
void insertQcqpValue(Key key, const Pose2& value, Values& qcqpValues) {
  InsertQcqpValue<Pose2, 1>(key, value, &qcqpValues);
}

/* ************************************************************************* */
void insertQcqpValue(Key key, const Pose3& value, Values& qcqpValues) {
  InsertQcqpValue<Pose3, 1>(key, value, &qcqpValues);
}

/* ************************************************************************* */
Rot2 rot2FromQcqpValue(const Matrix& value) {
  return traits<Rot2>::FromQcqpValue<1>(value);
}

/* ************************************************************************* */
Rot3 rot3FromQcqpValue(const Matrix& value) {
  return traits<Rot3>::FromQcqpValue<1>(value);
}

/* ************************************************************************* */
Pose2 pose2FromQcqpValue(const Matrix& value) {
  return traits<Pose2>::FromQcqpValue<1>(value);
}

/* ************************************************************************* */
Pose3 pose3FromQcqpValue(const Matrix& value) {
  return traits<Pose3>::FromQcqpValue<1>(value);
}

/* ************************************************************************* */
Values extractQcqpValuesRot2(const Values& qcqpValues) {
  return ExtractTypedQcqpValues<Rot2>(qcqpValues);
}

/* ************************************************************************* */
Values extractQcqpValuesRot3(const Values& qcqpValues) {
  return ExtractTypedQcqpValues<Rot3>(qcqpValues);
}

/* ************************************************************************* */
Values extractQcqpValuesPose2(const Values& qcqpValues) {
  return ExtractTypedQcqpValues<Pose2>(qcqpValues);
}

/* ************************************************************************* */
Values extractQcqpValuesPose3(const Values& qcqpValues) {
  return ExtractTypedQcqpValues<Pose3>(qcqpValues);
}

/* ************************************************************************* */
void QcqpProblem::addConstraint(const LinearConstraint& constraint) {
  if (constraint.isEquality()) {
    eqConstraints_.push_back(constraint.createEqualityFactor());
  } else {
    ineqConstraints_.push_back(constraint.createInequalityFactor());
  }
}

/* ************************************************************************* */
void QcqpProblem::addConstraint(const QuadraticConstraint& constraint) {
  if (constraint.isEquality()) {
    eqConstraints_.push_back(constraint.createEqualityFactor());
  } else {
    ineqConstraints_.push_back(constraint.createInequalityFactor());
  }
}

}  // namespace gtsam

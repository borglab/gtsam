/**
 * @file   GradientDescentOptimizer.cpp
 * @brief
 * @author ydjian
 * @date   Jun 11, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

class GradientDescentOptimizer;

class GradientDescentParams : public NonlinearOptimizerParams {

public:
  typedef NonlinearOptimizerParams Base;

  GradientDescentParams():Base() {}
  virtual void print(const std::string& str = "") const {
    Base::print(str);
  }

  virtual ~GradientDescentParams() {}
};

class GradientDescentState : public NonlinearOptimizerState {

public:

  typedef NonlinearOptimizerState Base;

  GradientDescentState(const NonlinearFactorGraph& graph, const Values& values)
    : Base(graph, values) {}
};

class GradientDescentOptimizer : public NonlinearOptimizer {

public:

  typedef boost::shared_ptr<GradientDescentOptimizer> shared_ptr;
  typedef NonlinearOptimizer Base;
  typedef GradientDescentState States;
  typedef GradientDescentParams Parameters;

protected:

  Parameters params_;
  States state_;
  Ordering::shared_ptr ordering_;
  VectorValues::shared_ptr gradient_;

public:

  GradientDescentOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues, const Parameters& params = Parameters())
    : Base(graph), params_(params), state_(graph, initialValues),
      ordering_(initialValues.orderingArbitrary()),
      gradient_(new VectorValues(initialValues.zeroVectors(*ordering_))) {}

  virtual ~GradientDescentOptimizer() {}

  virtual void iterate();

protected:

  virtual const NonlinearOptimizerState& _state() const { return state_; }
  virtual const NonlinearOptimizerParams& _params() const { return params_; }
};



}

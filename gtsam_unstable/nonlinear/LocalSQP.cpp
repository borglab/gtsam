/*
 * SQPLineSearch2.h
 * @brief:
 * @date: Aug 26, 2016
 * @author: Ivan Dario Jimenez
 */

#include <gtsam_unstable/nonlinear/LocalSQP.h>

namespace gtsam {
LocalSQP::State LocalSQP::iterate(const State& currentState) const {
  return State();
}

Values LocalSQP::optimize(const Values &initials, unsigned int max_iter) const {
  VectorValues duals;
  for (unsigned int index = 0; index < program_.equalities.size(); index++) {
//    duals.insert(program_.equalities[index]->dualKey(),
//        zero(program_.equalities[index]->dim()));
  }
  State initial(initials, duals);
  return Values();
}

}

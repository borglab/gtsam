/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam/sam/RISAMGraduatedFactor.h>

namespace gtsam {
/* ************************************************************************* */
GraduatedFactor::GraduatedFactor(GraduatedFactor::RobustLoss::shared_ptr loss,
                                 GraduationScheduler::shared_ptr scheduler)
    : robust_loss_(loss), scheduler_(scheduler) {
  mu_ = std::make_shared<double>(scheduler_->muInit());
}

/* ************************************************************************* */
GraduatedFactor::GraduatedFactor(const GraduatedFactor& other)
    : robust_loss_(other.robust_loss_), scheduler_(other.scheduler_) {
  mu_ = std::make_shared<double>(*(other.mu_));
}

/* ************************************************************************* */
const GraduatedFactor::RobustLoss::shared_ptr GraduatedFactor::loss() const {
  return robust_loss_;
}
/* ************************************************************************* */
const GraduationScheduler::shared_ptr GraduatedFactor::scheduler() const {
  return scheduler_;
}

}  // namespace gtsam

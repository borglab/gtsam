/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


#include "risam/GraduatedFactor.h"

namespace risam {
/* ************************************************************************* */
GraduatedFactor::GraduatedFactor(GraduatedKernel::shared_ptr kernel) : kernel_(kernel) {
  mu_ = std::make_shared<double>(kernel_->muInit());
}

/* ************************************************************************* */
GraduatedFactor::GraduatedFactor(const GraduatedFactor& other) : kernel_(other.kernel_) {
  mu_ = std::make_shared<double>(*(other.mu_));
}

/* ************************************************************************* */
const GraduatedKernel::shared_ptr GraduatedFactor::kernel() const { return kernel_; }

/* ************************************************************************* */
void GraduatedFactor::updateKernel(const GraduatedKernel::shared_ptr& new_kernel) {
  kernel_ = new_kernel;
  *mu_ = new_kernel->muInit();
}

}  // namespace risam
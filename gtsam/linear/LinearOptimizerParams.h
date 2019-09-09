//
// Created by fan on 9/8/19.
//

#ifndef GTSAM_LINEAROPTIMIZERPARAMS_H
#define GTSAM_LINEAROPTIMIZERPARAMS_H

namespace gtsam {

    /** See NonlinearOptimizerParams::linearSolverType */
    typedef enum LinearSolverType {
        MULTIFRONTAL_CHOLESKY,
        MULTIFRONTAL_QR,
        SEQUENTIAL_CHOLESKY,
        SEQUENTIAL_QR,
        Iterative, /* Experimental Flag */
        CHOLMOD, /* Experimental Flag */
        EIGEN_QR,
        EIGEN_CHOLESKY,
    } LinearSolverType;
}


class LinearOptimizerParams {

};


#endif //GTSAM_LINEAROPTIMIZERPARAMS_H

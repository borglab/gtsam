/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IrwinHall.cpp
 * @brief Irwin-Hall probability density kernels.
 * @author Brett Downing
 */

#include <gtsam/basis/IrwinHall.h>
namespace gtsam {
namespace kernels {

/**
 * coefficients for the Irwin Hall Probability Density Functions
 * https://oeis.org/A188816
 */

const PiecewisePolynomial<0, 1> IrwinHall0({{1.}, {0., 1.}, 1. / 2.});
const PiecewisePolynomial<1, 2> IrwinHall1({{0., 1., 2., -1.},
                                            {0., 1., 2.},
                                            1.});
const PiecewisePolynomial<2, 3> IrwinHall2(
    {{0., 0., 1. / 2., -3. / 2., 6. / 2., -2. / 2., 9. / 2., -6. / 2., 1. / 2.},
     {0., 1., 2., 3.},
     3. / 2.});
const PiecewisePolynomial<3, 4> IrwinHall3(
    {{0., 0., 0., 1. / 6., 2. / 3., -2., 2., -1. / 2., -22. / 3., 10., -4.,
      1. / 2., 32. / 3., -8., 2., -1. / 6.},
     {0., 1., 2., 3., 4.},
     2.});
const PiecewisePolynomial<4, 5> IrwinHall4(
    {{0.,          0.,         0.,        0.,       1. / 24.,
      -5. / 24.,   5. / 6.,    -5. / 4.,  5. / 6.,  -1. / 6.,
      155. / 24.,  -25. / 2.,  35. / 4.,  -5. / 2., 1. / 4.,
      -655. / 24., 65. / 2.,   -55. / 4., 5. / 2.,  -1. / 6.,
      625. / 24.,  -125. / 6., 25. / 4.,  -5. / 6., 1. / 24.},
     {0., 1., 2., 3., 4., 5.},
     5. / 2.});
const PiecewisePolynomial<5, 6> IrwinHall5(
    {{0.,           0.,         0.,        0.,        0.,      1. / 120.,
      1. / 20.,     -1. / 4.,   1. / 2.,   -1. / 2.,  1. / 4., -1. / 24.,
      -79. / 20.,   39. / 4.,   -19. / 2., 9. / 2.,   -1.,     1. / 12.,
      731. / 20.,   -231. / 4., 71. / 2.,  -21. / 2., 3. / 2., -1. / 12.,
      -1829. / 20., 409. / 4.,  -89. / 2., 19. / 2.,  -1.,     1. / 24.,
      324. / 5.,    -54.,       18.,       -3.,       1. / 4., -1. / 120.},
     {0., 1., 2., 3., 4., 5., 6.},
     3.});
const PiecewisePolynomial<6, 7> IrwinHall6({{0.,
                                             0.,
                                             0.,
                                             0.,
                                             0.,
                                             0.,
                                             1. / 720.,
                                             -7. / 720.,
                                             7. / 120.,
                                             -7. / 48.,
                                             7. / 36.,
                                             -7. / 48.,
                                             7. / 120.,
                                             -1. / 120.,
                                             1337. / 720.,
                                             -133. / 24.,
                                             329. / 48.,
                                             -161. / 36.,
                                             77. / 48.,
                                             -7. / 24.,
                                             1. / 48.,
                                             -12089. / 360.,
                                             196. / 3.,
                                             -1253. / 24.,
                                             196. / 9.,
                                             -119. / 24.,
                                             7. / 12.,
                                             -1. / 36.,
                                             59591. / 360.,
                                             -700. / 3.,
                                             3227. / 24.,
                                             -364. / 9.,
                                             161. / 24.,
                                             -7. / 12.,
                                             1. / 48.,
                                             -208943. / 720.,
                                             7525. / 24.,
                                             -6671. / 48.,
                                             1169. / 36.,
                                             -203. / 48.,
                                             7. / 24.,
                                             -1. / 120.,
                                             117649. / 720.,
                                             -16807. / 120.,
                                             2401. / 48.,
                                             -343. / 36.,
                                             49. / 48.,
                                             -7. / 120.,
                                             1. / 720.},
                                            {0., 1., 2., 3., 4., 5., 6., 7.},
                                            7. / 2.});

}  // namespace kernels
}  // namespace gtsam

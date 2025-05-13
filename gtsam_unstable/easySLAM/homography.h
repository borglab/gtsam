/**
 * @file    homography.h
 * @brief   Homography utilities
 * @author  Alireza Fathi
 */

#pragma once

#include <gtsam/Pose3.h>

/**
 * recieves the m matrix of marker corner pixel locations in image and
 * M matrix of marker corner locations in marker coordinate frame and
 * returns the L matrix on which later we do SVD to get the homography
 * @param m 4*2 m contains the 4 corner points of the marker
 * @param M 4*3 M contains the 4 corner points of the marker in marker
 * coordinate frame
 * @return 8by9 matrix L
 */
Matrix getL(const Matrix& m, const Matrix& M);

/**
 * get the homography from matrix L
 * @param L 8by9 matrix L
 * @return 3*3 homography
 */
Matrix getH(const Matrix& L);

/**
 * get the homography from correspondences
 * @param m 4*2 m contains the 4 corner points of the marker
 * @param M 4*3 M contains the 4 corner points of the marker in marker
 * coordinate frame
 * @return 3*3 homography
 */
Matrix getH(const Matrix& m, const Matrix& M);

// Matrix efficientGetH(const Matrix& m/*4by2*/, const Matrix& M/*4by3*/);

/**
 * transformation from marker to camera
 * @param H 3*3 homography
 * @param K 3*3 calibration
 * @return 4*4 transformation from marker to camera (cTm)
 */
gtsam::Pose3 getTransformationFromMarkerToCamera(const Matrix& H,
                                                 const Matrix& K);

/**
 * Given a supposed to be a rotation matrix Q (probably computed by the
 * homography method of Zhang), which is not treating like a rotation matrix
 * since its determinant is not 1 and QQ' is not equal to I, this function finds
 * the best rotation matrix that approximates Q. How? it is described in
 * Zhang98tr.
 */
gtsam::Rot3 fixToRotation_zhang(const gtsam::Rot3& Q);

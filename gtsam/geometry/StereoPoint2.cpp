/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * StereoPoint2.cpp
 *
 *  Created on: Jan 26, 2010
 *      Author: dellaert
 */

#include <iostream>
#include <gtsam/geometry/StereoPoint2.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
void StereoPoint2::print(const string& s) const {
	cout << s << "(" << uL_ << ", " << uR_ << ", " << v_ << ")"	<< endl;
}
/* ************************************************************************* */

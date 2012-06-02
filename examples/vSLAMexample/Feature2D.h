/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Feature2D.h
 * @brief
 * @author Duy-Nguyen Ta
 */

#pragma once

#include "gtsam/geometry/Point2.h"
#include <iostream>

struct Feature2D
{

  gtsam::Point2 m_p;
  int m_idCamera;         // id of the camera pose that makes this measurement
  int m_idLandmark;       // id of the 3D landmark that it is associated with

  Feature2D(int idCamera, int idLandmark, gtsam::Point2 p) :
    m_p(p),
    m_idCamera(idCamera),
    m_idLandmark(idLandmark) {}

  void print(const std::string& s = "") const {
    std::cout << s << std::endl;
    std::cout << "Pose id: " << m_idCamera << " -- Landmark id: "  << m_idLandmark << std::endl;
    m_p.print("\tMeasurement: ");
  }

};

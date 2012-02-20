/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file StringFormatter.h
 * @brief 
 * @author Richard Roberts
 * @date Feb 19, 2012
 */

#pragma once

namespace gtsam {

  /**
   *
   */
  class StringFormatter {

  public:

    virtual ~StringFormatter() {}

    virtual std::string keyToString(Key key) = 0;

  };

} /* namespace gtsam */

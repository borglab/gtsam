/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     types.h
 * @brief    Typedefs for easier changing of types
 * @author   Richard Roberts
 * @date     Aug 21, 2010
 * @addtogroup base
 */

#include <boost/lexical_cast.hpp>

#include <gtsam/base/types.h>

namespace gtsam {

  std::string _defaultIndexFormatter(Index j) {
    return boost::lexical_cast<std::string>(j);
  }

}
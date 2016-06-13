/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief common macros used by handwritten exports of the python module
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

#pragma once

 /* Fix to avoid registration warnings */
// Solution taken from https://github.com/BVLC/caffe/pull/4069/commits/673e8cfc0b8f05f9fa3ebbad7cc6202822e5d9c5 
#define REGISTER_SHARED_PTR_TO_PYTHON(PTR) do { \
  const boost::python::type_info info = \
    boost::python::type_id<boost::shared_ptr<PTR > >(); \
  const boost::python::converter::registration* reg = \
    boost::python::converter::registry::query(info); \
  if (reg == NULL) { \
    boost::python::register_ptr_to_python<boost::shared_ptr<PTR > >(); \
  } else if ((*reg).m_to_python == NULL) { \
    boost::python::register_ptr_to_python<boost::shared_ptr<PTR > >(); \
  } \
} while (0)

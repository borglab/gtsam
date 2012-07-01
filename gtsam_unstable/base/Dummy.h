/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Dummy.h
 * @brief Dummy class for testing MATLAB memory allocation
 * @author Andrew Melim
 * @author Frank Dellaert
 * @date June 14, 2012
 */

namespace gtsam {

  static size_t gDummyCount;

  struct Dummy {
    size_t id;
    Dummy():id(++gDummyCount) {
      std::cout << "Dummy constructor " << id << std::endl;
    }
    ~Dummy() {
      std::cout << "Dummy destructor " << id << std::endl;
    }
    void print(const std::string& s="") const {
      std::cout << s << "Dummy " << id << std::endl;
    }

    unsigned char dummyTwoVar(unsigned char a) const {
        return a;
    }

  };

} // namespace gtsam


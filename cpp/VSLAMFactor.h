/**
 * @file    VSLAMFactor.h
 * @brief   A Nonlinear Factor, specialized for visual SLAM
 * @author  Alireza Fathi
 */

#pragma once

#include "NonlinearFactor.h"
#include "LinearFactor.h"
#include "Cal3_S2.h"

namespace gtsam {

class VSLAMConfig;

/**
 * Non-linear factor for a constraint derived from a 2D measurement,
 * i.e. the main building block for visual SLAM.
 */
class VSLAMFactor : public NonlinearFactor<VSLAMConfig>
{
 private:

	int cameraFrameNumber_, landmarkNumber_;
  std::string cameraFrameName_, landmarkName_;
  Cal3_S2 K_; // Calibration stored in each factor. FD: need to think about this.
  typedef gtsam::NonlinearFactor<VSLAMConfig> ConvenientFactor;

 public:

  typedef boost::shared_ptr<VSLAMFactor> shared_ptr; // shorthand for a smart pointer to a factor

  /**
   * Constructor
   * @param z is the 2 dimensional location of point in image (the measurement)
   * @param sigma is the standard deviation
   * @param cameraFrameNumber is basically the frame number
   * @param landmarkNumber is the index of the landmark
   * @param K the constant calibration
   */
  VSLAMFactor(const Vector& z, double sigma, int cameraFrameNumber, int landmarkNumber, const Cal3_S2& K);


  /**
   * print
   * @param s optional string naming the factor
   */
  void print(const std::string& s="VSLAMFactor") const;

  /**
   * equals
   */
  bool equals(const NonlinearFactor<VSLAMConfig>&, double tol=1e-9) const;

  /**
   * predict the measurement
   */
  Vector predict(const VSLAMConfig&) const;

  /**
   * calculate the error of the factor
   */
  Vector error_vector(const VSLAMConfig&) const;

  /**
   * linerarization
   */
  LinearFactor::shared_ptr linearize(const VSLAMConfig&) const;

  int getCameraFrameNumber() const { return cameraFrameNumber_; }
  int getLandmarkNumber()    const { return landmarkNumber_;    }
};

}

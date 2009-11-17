/**
 * @file    VSLAMFactor.h
 * @brief   A Nonlinear Factor, specialized for visual SLAM
 * @author  Alireza Fathi
 */

#pragma once

#include "NonlinearFactor.h"
#include "GaussianFactor.h"
#include "Cal3_S2.h"
#include "Testable.h"

namespace gtsam {

class VSLAMConfig;

/**
 * Non-linear factor for a constraint derived from a 2D measurement,
 * i.e. the main building block for visual SLAM.
 */
class VSLAMFactor : public NonlinearFactor<VSLAMConfig>, Testable<VSLAMFactor>
{
private:

	int cameraFrameNumber_, landmarkNumber_;
	std::string cameraFrameName_, landmarkName_;
	boost::shared_ptr<Cal3_S2> K_; // Calibration stored in each factor. FD: need to think about this.
	typedef NonlinearFactor<VSLAMConfig> ConvenientFactor;

public:

	typedef boost::shared_ptr<VSLAMFactor> shared_ptr; // shorthand for a smart pointer to a factor
	typedef boost::shared_ptr<Cal3_S2> shared_ptrK;

	/**
	 * Default constructor
	 */
	VSLAMFactor();

	/**
	 * Constructor
	 * @param z is the 2 dimensional location of point in image (the measurement)
	 * @param sigma is the standard deviation
	 * @param cameraFrameNumber is basically the frame number
	 * @param landmarkNumber is the index of the landmark
	 * @param K the constant calibration
	 */
	VSLAMFactor(const Point2& z, double sigma, int cameraFrameNumber, int landmarkNumber, const shared_ptrK & K);


	/**
	 * print
	 * @param s optional string naming the factor
	 */
	void print(const std::string& s="VSLAMFactor") const;

	/**
	 * equals
	 */
	bool equals(const VSLAMFactor&, double tol=1e-9) const;

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
	GaussianFactor::shared_ptr linearize(const VSLAMConfig&) const;

	int getCameraFrameNumber() const { return cameraFrameNumber_; }
	int getLandmarkNumber()    const { return landmarkNumber_;    }

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(cameraFrameNumber_);
		ar & BOOST_SERIALIZATION_NVP(landmarkNumber_);
		ar & BOOST_SERIALIZATION_NVP(cameraFrameName_);
		ar & BOOST_SERIALIZATION_NVP(landmarkName_);
		ar & BOOST_SERIALIZATION_NVP(K_);
	}
};

}

#ifndef FEATURE2D_H
#define FEATURE2D_H

#include "gtsam/geometry/Point2.h"
#include <iostream>

class Feature2D
{
public:
    gtsam::Point2 m_p;
		int m_idCamera;						// id of the camera pose that makes this measurement
		int m_idLandmark;       // id of the 3D landmark that it is associated with
public:
		Feature2D(int idCamera, int idLandmark, gtsam::Point2 p)
						:m_idCamera(idCamera),
						m_idLandmark(idLandmark),
						m_p(p)
		{};

    void print(const std::string& s = "") const
    {
        std::cout << s << std::endl;
				std::cout << "Pose id: " << m_idCamera << " -- Landmark id: "  << m_idLandmark << std::endl;
				m_p.print("\tMeasurement: ");
    }
};

#endif // FEATURE2D_H

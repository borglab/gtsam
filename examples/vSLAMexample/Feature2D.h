#ifndef FEATURE2D_H
#define FEATURE2D_H

#include "gtsam/geometry/Point2.h"
#include <iostream>

class Feature2D
{
public:
    gtsam::Point2 m_p;
    int m_id;       // id of the 3D landmark that it is associated with
public:
    Feature2D(int id, gtsam::Point2 p):m_id(id), m_p(p) {};

    void print(const std::string& s = "") const
    {
        std::cout << s << std::endl;
        std::cout << "Id: " << m_id << std::endl;
        m_p.print();
    }
};

#endif // FEATURE2D_H

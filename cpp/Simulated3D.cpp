/**
* @file   Simulated3D.cpp
* @brief  measurement functions and derivatives for simulated 3D robot
* @author Alex Cunningham
**/

#include "Simulated3D.h"

namespace gtsam {

Vector prior_3D (const Vector& x)
{
	return x;
}

Matrix Dprior_3D(const Vector& x)
{
	Matrix H = eye((int) x.size());
	return H;
}

Vector odo_3D(const Vector& x1, const Vector& x2)
{
	return x2 - x1;
}

Matrix Dodo1_3D(const Vector& x1, const Vector& x2)
{
	Matrix H = -1 * eye((int) x1.size());
	return H;
}

Matrix Dodo2_3D(const Vector& x1, const Vector& x2)
{
	Matrix H = eye((int) x1.size());
	return H;
}


Vector mea_3D(const Vector& x,  const Vector& l)
{
	return l - x;
}

Matrix Dmea1_3D(const Vector& x, const Vector& l)
{
	Matrix H = -1 * eye((int) x.size());
	return H;
}

Matrix Dmea2_3D(const Vector& x, const Vector& l)
{
	Matrix H = eye((int) x.size());
	return H;
}

} // namespace gtsam
/*
 * @file NonlinearEquality.h
 * @brief Factor to handle enforced equality between factors
 * @author Alex Cunningham
 */

#pragma once

#include "NonlinearFactor.h"

namespace gtsam {

/**
 * An equality factor that forces either one variable to a constant,
 * or a set of variables to be equal to each other.
 * Throws an error at linearization if the constraints are not met.
 */
template<class Config>
class NonlinearEquality : public NonlinearFactor<Config>{
public:
	NonlinearEquality();
	virtual ~NonlinearEquality();
};

}


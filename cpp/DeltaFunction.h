/*
 * DeltaFunction.h
 *
 *  Created on: Aug 11, 2009
 *      Author: alexgc
 */

#ifndef DELTAFUNCTION_H_
#define DELTAFUNCTION_H_

#include <string>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include "Vector.h"

namespace gtsam {

class DeltaFunction : boost::noncopyable {
protected:
	Vector value_; /// location of the delta function
	std::string key_; /// id of node with delta function

public:
	typedef boost::shared_ptr<DeltaFunction> shared_ptr;

	/**
	 * Default Constructor
	 */
	DeltaFunction();

	/**
	 * Constructor with initialization
	 */
	DeltaFunction(const Vector& value, const std::string& key);

	/**
	 * Copy constructor
	 */
	DeltaFunction(const DeltaFunction& df);

	virtual ~DeltaFunction();

	/**
	 * basic get functions
	 */
	Vector get_value() const {return value_;}
	std::string get_key() const {return key_;}


	/** equals function */
	bool equals(const DeltaFunction &cg) const;

	/** basic print */
	void print() const;

};

/** equals function for testing - prints if not equal */
bool assert_equal(const DeltaFunction& actual, const DeltaFunction& expected, double tol=1e-9);

}

#endif /* DELTAFUNCTION_H_ */

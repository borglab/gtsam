/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Testable.h
 * @brief   Concept check for values that can be used in unit tests
 * @author  Frank Dellaert
 * 
 * The necessary functions to implement for Testable are defined
 * below with additional details as to the interface.
 * The concept checking function will check whether or not
 * the function exists in derived class and throw compile-time errors.
 * 
 * print with optional string naming the object
 * 		void print(const std::string& name) const = 0;
 * 
 * equality up to tolerance
 * tricky to implement, see NoiseModelFactor1 for an example
 * equals is not supposed to print out *anything*, just return true|false
 * 		bool equals(const Derived& expected, double tol) const = 0;
 * 
 */

// \callgraph

#pragma once

#include <boost/shared_ptr.hpp>
#include <stdio.h>
#include <string>

#define GTSAM_PRINT(x)((x).print(#x))

namespace gtsam {

	/**
	 * A testable concept check that should be placed in applicable unit
	 * tests and in generic algorithms.
	 *
	 * See macros for details on using this structure
	 * @addtogroup base
	 * @tparam T is the type this constrains to be testable - assumes print() and equals()
	 */
	template <class T>
	class TestableConcept {
	  static bool checkTestableConcept(const T& d) {
	  	// check print function, with optional string
	    d.print(std::string());
	    d.print();

	    // check print, with optional threshold
	    double tol = 1.0;
	    bool r1 = d.equals(d, tol);
	    bool r2 = d.equals(d);
	    return r1 && r2;
	  }
	};

	/** Call print on the object */
	template<class T>
	inline void print(const T& object, const std::string& s = "") {
		object.print(s);
	}

	/** Call equal on the object */
	template<class T>
	inline bool equal(const T& obj1, const T& obj2, double tol) {
		return obj1.equals(obj2, tol);
	}

	/** Call equal on the object without tolerance (use default tolerance) */
	template<class T>
	inline bool equal(const T& obj1, const T& obj2) {
		return obj1.equals(obj2);
	}

	/**
	 * This template works for any type with equals
	 */
	template<class V>
	bool assert_equal(const V& expected, const V& actual, double tol = 1e-9) {
		if (actual.equals(expected, tol))
			return true;
		printf("Not equal:\n");
		expected.print("expected");
		actual.print("actual");
		return false;
	}

	/**
	 * Template to create a binary predicate
	 */
	template<class V>
	struct equals : public std::binary_function<const V&, const V&, bool> {
		double tol_;
		equals(double tol = 1e-9) : tol_(tol) {}
		bool operator()(const V& expected, const V& actual) {
			return (actual.equals(expected, tol_));
		}
	};

	/**
	 * Binary predicate on shared pointers
	 */
	template<class V>
	struct equals_star : public std::binary_function<const boost::shared_ptr<V>&, const boost::shared_ptr<V>&, bool> {
		double tol_;
		equals_star(double tol = 1e-9) : tol_(tol) {}
		bool operator()(const boost::shared_ptr<V>& expected, const boost::shared_ptr<V>& actual) {
			if (!actual || !expected) return false;
			return (actual->equals(*expected, tol_));
		}
	};

} // \namespace gtsam

/**
 * Macros for using the TestableConcept
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 */
#define GTSAM_CONCEPT_TESTABLE_INST(T) template class gtsam::TestableConcept<T>;
#define GTSAM_CONCEPT_TESTABLE_TYPE(T) typedef gtsam::TestableConcept<T> _gtsam_TestableConcept_##T;

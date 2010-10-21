/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Testable
 * @brief   Abstract base class for values that can be used in unit tests
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
 * tricky to implement, see NonlinearFactor1 for an example
 * equals is not supposed to print out *anything*, just return true|false
 * 		bool equals(const Derived& expected, double tol) const = 0;
 * 
 */

// \callgraph

#pragma once

#include <boost/shared_ptr.hpp>
#include <stdio.h>

#define GTSAM_PRINT(x)((x).print(#x))

namespace gtsam {

  /**
   * The Testable class should be templated with the derived class, e.g.
   * class Rot3 : public Testable<Rot3>. This allows us to define the
   * input type of equals as a Rot3 as well.
   */
	template <class Derived>
	class Testable {

	private:
		/**
		 * This concept check is to make sure these methods exist in derived classes
		 * This is as an alternative to declaring those methods virtual, which is slower
		 */
	  static void concept(const Derived& d) {
	    const Derived *const_derived = static_cast<Derived*>(&d);
	    const_derived->print(std::string());
	    const_derived->print();
	    bool r;
	    r = const_derived->equals(*const_derived, 1.0);
	    r = const_derived->equals(*const_derived);
	  }

	}; // Testable class

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
			return (actual->equals(*expected, tol_));
		}
	};

} // gtsam

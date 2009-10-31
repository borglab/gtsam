/**
 * @file    Testable
 * @brief   Abstract base class for values that can be used in unit tests
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <boost/shared_ptr.hpp>

namespace gtsam {

  /**
   * The Testable class should be templated with the derived class, e.g.
   * class Rot3 : public Testable<Rot3>. This allows us to define the
   * input type of equals as a Rot3 as well.
   */
	template <class Derived>
	class Testable {

	public:
	    virtual ~Testable() {};

		/**
		 * print
		 * @param s optional string naming the object
		 */
		virtual void print(const std::string& name) const = 0;

		/**
		 * equality up to tolerance
		 * tricky to implement, see NonLinearFactor1 for an example
		 * equals is not supposed to print out *anything*, just return true|false
		 */
		virtual bool equals(const Derived& expected, double tol) const = 0;

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
	bool equals(const V& expected, const V& actual, double tol = 1e-9) {
		return (actual.equals(expected, tol));
	}

	/**
	 * Binary predicate on shared pointers
	 */
	template<class V>
	bool equals_star(const boost::shared_ptr<V>& expected,
			const boost::shared_ptr<V>& actual, double tol = 1e-9) {
		return (actual->equals(*expected, tol));
	}

} // gtsam

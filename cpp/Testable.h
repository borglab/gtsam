/**
 * @file    Testable
 * @brief   Abstract base class for values that can be used in unit tests
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

namespace gtsam {

  /**
   * The Testable class should be templated with the derived class, e.g.
   * class Rot3 : public Testable<Rot3>. This allows us to define the
   * input type of equals as a Rot3 as well.
   */
	template <class Derived>
	class Testable {

	public:

		/**
		 * print
		 * @param s optional string naming the object
		 */
		virtual void print(const std::string& name) const = 0;

		/**
		 * equality up to tolerance
		 * tricky to implement, see NonLinearFactor1 for an example
		 */
		virtual bool equals(const Derived& expected, double tol) const = 0;

	}; // Testable class

	/**
	 * This template works for any type with equals
	 */
	template<class V>
	bool assert_equal(const V& actual, const V& expected, double tol = 1e-9) {
		if (actual.equals(expected, tol))
			return true;
		printf("Not equal:\n");
		actual.print("actual");
		expected.print("expected");
		return false;
	}

} // gtsam

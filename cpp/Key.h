/*
 * Key.h
 *
 *  Created on: Jan 12, 2010
 *  @author: Frank Dellaert
 *  @author: Richard Roberts
 */

#pragma once

#include <boost/format.hpp>
#include <boost/serialization/serialization.hpp>

namespace gtsam {

	/**
	 * Symbol key class is templated on
	 * 1) the type T it is supposed to retrieve, for extra type checking
	 * 2) the character constant used for its string representation
	 * TODO: make Testable
	 */
	template <class T, char C>
	class Symbol {

	private:
		size_t j_;

	public:

		// Constructors:

		Symbol():j_(999999) {}
		Symbol(size_t j):j_(j) {}

		// Get stuff:

		size_t index() const { return j_;}
		const char* c_str() const { return (std::string)(*this).c_str();}
		operator std::string() const { return (boost::format("%c%d") % C % j_).str(); }
		std::string latex() const { return (boost::format("%c_{%d}") % C % j_).str(); }

		// logic:

		bool operator< (const Symbol& compare) const { return j_<compare.j_;}
		bool operator== (const Symbol& compare) const { return j_==compare.j_;}
		int compare(const Symbol& compare) const {return j_-compare.j_;}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(j_);
		}

	};

} // namespace gtsam


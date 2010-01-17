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
#include <boost/lexical_cast.hpp>
#include <boost/integer_traits.hpp>

#define ALPHA '\224'

namespace gtsam {

	/**
	 * TypedSymbol key class is templated on
	 * 1) the type T it is supposed to retrieve, for extra type checking
	 * 2) the character constant used for its string representation
	 * TODO: make Testable
	 */
	template <class T, char C>
	class TypedSymbol {

	private:
		size_t j_;

	public:

		// Constructors:

		TypedSymbol():j_(boost::integer_traits<size_t>::const_max) {}
		TypedSymbol(size_t j):j_(j) {}

		// Get stuff:

		size_t index() const { return j_;}
		const char* c_str() const { return (std::string)(*this).c_str();}
		operator std::string() const { return (boost::format("%c%d") % C % j_).str(); }
		std::string latex() const { return (boost::format("%c_{%d}") % C % j_).str(); }

		// logic:

		bool operator< (const TypedSymbol& compare) const { return j_<compare.j_;}
		bool operator== (const TypedSymbol& compare) const { return j_==compare.j_;}
		int compare(const TypedSymbol& compare) const {return j_-compare.j_;}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(j_);
		}
	};


	/**
	 * Character and index key used in VectorConfig, GaussianFactorGraph,
	 * GaussianFactor, etc.  These keys are generated at runtime from TypedSymbol
	 * keys when linearizing a nonlinear factor graph.  This key is not type
	 * safe, so cannot be used with any Nonlinear* classes.
	 *
	 * richard:  temporarily named 'Symbol' to make refactoring easier
	 */
	class Symbol {
	private:
	  unsigned char c_;
	  size_t j_;

	public:
	  /** Default constructor */
	  Symbol() : c_(0), j_(boost::integer_traits<size_t>::const_max) {}

	  /** Copy constructor */
	  Symbol(const Symbol& key) : c_(key.c_), j_(key.j_) {}

	  /** Constructor */
	  Symbol(unsigned char c, size_t j): c_(c), j_(j) {}

	  /** Casting constructor from TypedSymbol */
	  template<class T, char C>
	  Symbol(const TypedSymbol<T,C>& symbol): c_(C), j_(symbol.index()) {}

	  /** "Magic" key casting constructor from string */
#ifdef GTSAM_MAGIC_KEY
	  Symbol(const std::string& str) {
	    if(str.length() < 1)
	      throw std::invalid_argument("Cannot parse string key '" + str + "'");
	    else {
	      const char *c_str = str.c_str();
	      c_ = c_str[0];
	      if(str.length() > 1)
	        j_ = boost::lexical_cast<size_t>(c_str+1);
	      else
	        j_ = 0;
	    }
	  }

	  Symbol(const char *c_str) {
	    std::string str(c_str);
      if(str.length() < 1)
        throw std::invalid_argument("Cannot parse string key '" + str + "'");
      else {
        c_ = c_str[0];
        if(str.length() > 1)
          j_ = boost::lexical_cast<size_t>(c_str+1);
        else
          j_ = 0;
      }
	  }
#endif

	  /** Retrieve key character */
	  unsigned char chr() const { return c_; }

	  /** Retrieve key index */
	  size_t index() const { return j_; }

	  /** Create a string from the key */
	  operator std::string() const { return str(boost::format("%c%d") % c_ % j_); }

	  /** Comparison for use in maps */
	  bool operator< (const Symbol& comp) const { return c_ < comp.c_ || (comp.c_ == c_ && j_ < comp.j_); }
	  bool operator== (const Symbol& comp) const { return comp.c_ == c_ && comp.j_ == j_; }
	  bool operator!= (const Symbol& comp) const { return comp.c_ != c_ || comp.j_ != j_; }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(c_);
      ar & BOOST_SERIALIZATION_NVP(j_);
    }
	};

} // namespace gtsam


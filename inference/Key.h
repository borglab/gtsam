/*
 * Key.h
 *
 *  Created on: Jan 12, 2010
 *  @author: Frank Dellaert
 *  @author: Richard Roberts
 */

#pragma once

#include <list>
#include <iostream>
#include <boost/format.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#ifdef GTSAM_MAGIC_KEY
#include <boost/lexical_cast.hpp>
#endif

#include <gtsam/base/Testable.h>

#define ALPHA '\224'

namespace gtsam {

	/**
	 * TypedSymbol key class is templated on
	 * 1) the type T it is supposed to retrieve, for extra type checking
	 * 2) the character constant used for its string representation
	 */
	template <class T, char C>
	class TypedSymbol : Testable<TypedSymbol<T,C> > {

	protected:
		size_t j_;

	public:

		// typedefs
		typedef T Value_t;

		// Constructors:

		TypedSymbol():j_(0) {}
		TypedSymbol(size_t j):j_(j) {}

		// Get stuff:

		size_t index() const { return j_;}
		static char chr()  { return C; }
		const char* c_str() const { return (std::string)(*this).c_str();}
		operator std::string() const { return (boost::format("%c%d") % C % j_).str(); }
		std::string latex() const { return (boost::format("%c_{%d}") % C % j_).str(); }

		// logic:

		bool operator< (const TypedSymbol& compare) const { return j_<compare.j_;}
		bool operator== (const TypedSymbol& compare) const { return j_==compare.j_;}
		bool operator!= (const TypedSymbol& compare) const { return j_!=compare.j_;}
		int compare(const TypedSymbol& compare) const {return j_-compare.j_;}

		// Testable Requirements
		virtual void print(const std::string& s="") const {
			std::cout << s << ": " << (std::string)(*this) << std::endl;
		}
		bool equals(const TypedSymbol& expected, double tol=0.0) const { return (*this)==expected; }

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(j_);
		}
	};

	/** forward declaration to avoid circular dependencies */
	template<class T, char C, typename L>
	class TypedLabeledSymbol;

	/**
	 * Character and index key used in VectorConfig, GaussianFactorGraph,
	 * GaussianFactor, etc.  These keys are generated at runtime from TypedSymbol
	 * keys when linearizing a nonlinear factor graph.  This key is not type
	 * safe, so cannot be used with any Nonlinear* classes.
	 */
	class Symbol : Testable<Symbol> {
	protected:
	  unsigned char c_;
	  size_t j_;

	public:
	  /** Default constructor */
	  Symbol() : c_(0), j_(0) {}

	  /** Copy constructor */
	  Symbol(const Symbol& key) : c_(key.c_), j_(key.j_) {}

	  /** Constructor */
	  Symbol(unsigned char c, size_t j): c_(c), j_(j) {}

	  /** Casting constructor from TypedSymbol */
	  template<class T, char C>
	  Symbol(const TypedSymbol<T,C>& symbol): c_(C), j_(symbol.index()) {}

	  /** Casting constructor from TypedLabeledSymbol */
	  template<class T, char C, typename L>
	  Symbol(const TypedLabeledSymbol<T,C,L>& symbol): c_(C), j_(symbol.encode()) {}

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

		// Testable Requirements
		void print(const std::string& s="") const {
			std::cout << s << ": " << (std::string)(*this) << std::endl;
		}
		bool equals(const Symbol& expected, double tol=0.0) const { return (*this)==expected; }

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

	// Conversion utilities

	template<class Key> Symbol key2symbol(Key key) {
		return Symbol(key);
	}

	template<class Key> std::list<Symbol> keys2symbols(std::list<Key> keys) {
		std::list<Symbol> symbols;
		std::transform(keys.begin(), keys.end(), std::back_inserter(symbols), key2symbol<Key> );
		return symbols;
	}

	/**
	 * TypedLabeledSymbol is a variation of the TypedSymbol that allows
	 * for a runtime label to be placed on the label, so as to express
	 * "Pose 5 for robot 3"
	 * Labels should be kept to base datatypes (int, char, etc) to
	 * minimize cost of comparisons
	 *
	 * The labels will be compared first when comparing Keys, followed by the
	 * index
	 */
	template <class T, char C, typename L>
	class TypedLabeledSymbol : public TypedSymbol<T, C>, Testable<TypedLabeledSymbol<T,C,L> >  {

	protected:
		// Label
		L label_;

	public:

		typedef TypedSymbol<T, C> Base;

		// Constructors:

		TypedLabeledSymbol() {}
		TypedLabeledSymbol(size_t j, L label) : Base(j), label_(label) {}

		/** Constructor that decodes encoded labels */
		TypedLabeledSymbol(const Symbol& sym) : TypedSymbol<T,C>(0) {
			size_t shift = (sizeof(size_t)-sizeof(short)) * 8;
			this->j_ = (sym.index() << shift) >> shift; // truncate upper bits
			label_ = (L) (sym.index() >> shift); // remove lower bits
		}

		/** Constructor to upgrade an existing typed label with a label */
		TypedLabeledSymbol(const Base& key, L label) : Base(key.index()), label_(label) {}

		// Get stuff:

		L label() const { return label_;}
		const char* c_str() const { return (std::string)(*this).c_str();}
		operator std::string() const {
			std::string label_s = (boost::format("%1%") % label_).str();
			return (boost::format("%c%s_%d") % C % label_s % this->j_).str();
		}
		std::string latex() const {
			std::string label_s = (boost::format("%1%") % label_).str();
			return (boost::format("%c%s_{%d}") % C % label_s % this->j_).str();
		}

		// Needed for conversion to LabeledSymbol
		size_t convertLabel() const { return label_; }

		/**
		 * Encoding two numbers into a single size_t for conversion to Symbol
		 * Stores the label in the upper bytes of the index
		 */
		size_t encode() const {
			short label = (short) label_; //bound size of label to 2 bytes
			size_t shift = (sizeof(size_t)-sizeof(short)) * 8;
			size_t modifier = ((size_t) label) << shift;
			return this->j_ + modifier;
		}

		// logic:

		bool operator< (const TypedLabeledSymbol& compare) const {
			if (label_ == compare.label_) // sort by label first
				return this->j_<compare.j_;
			else
				return label_<compare.label_;
		}
		bool operator== (const TypedLabeledSymbol& compare) const
				{ return this->j_==compare.j_ && label_ == compare.label_;}
		int compare(const TypedLabeledSymbol& compare) const {
			if (label_ == compare.label_) // sort by label first
				return this->j_-compare.j_;
			else
				return label_-compare.label_;
		}

		// Testable Requirements
		void print(const std::string& s="") const {
					std::cout << s << ": " << (std::string)(*this) << std::endl;
				}
		bool equals(const TypedLabeledSymbol& expected, double tol=0.0) const
				{ return (*this)==expected; }

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(this->j_);
			ar & BOOST_SERIALIZATION_NVP(label_);
		}
	};
} // namespace gtsam


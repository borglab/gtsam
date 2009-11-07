/**
 * @file    Conditional.h
 * @brief   Base class for conditional densities
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <boost/utility.hpp> // for noncopyable
#include "Testable.h"

namespace gtsam {

  /** 
   * Base class for conditional densities
   *
   * We make it noncopyable so we enforce the fact that factors are
   * kept in pointer containers. To be safe, you should make them
   * immutable, i.e., practicing functional programming.
   */ 
  class Conditional : boost::noncopyable, public Testable<Conditional>
  {
  protected:

  	/** key of random variable */
  	std::string key_;

  public:

  	/** constructor */
  	Conditional(const std::string& key):key_(key) {}

  	/* destructor */
    virtual ~Conditional() {};

		/** check equality */
		bool equals(const Conditional& c, double tol = 1e-9) const {
			return key_ == c.key_;
		}

		/** return key */
    inline const std::string& key() const { return key_;}
		
    /** return parent keys */
    virtual std::list<std::string> parents() const = 0;
		
    /** return the number of parents */
    virtual std::size_t nrParents() const = 0;
  };

	// predicate to check whether a conditional has the sought key
	template<class Conditional>
	class onKey {
		const std::string& key_;
	public:
		onKey(const std::string& key):key_(key) {}
		bool operator()(const typename Conditional::shared_ptr& conditional) {
			return (conditional->key()==key_);
		}
	};


}

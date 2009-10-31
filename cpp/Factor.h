/**
 * @file    Factor.h
 * @brief   A simple factor class to use in a factor graph
 * @brief   factor
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <set>
#include <list>
#include <boost/utility.hpp> // for noncopyable
#include "Testable.h"

namespace gtsam {

	/** A combination of a key with a dimension - TODO FD: move, vector specific */
  struct Variable {
  private:
    std::string key_;
    std::size_t dim_;
  public:
  Variable(const std::string& key, std::size_t dim) : key_(key), dim_(dim) {}
    bool operator< (const Variable& other) const {return key_<other.key_; }
    const std::string& key() const { return key_;}
    std::size_t        dim() const { return dim_;}
  };

	/** A set of variables, used to eliminate linear factor factor graphs. TODO FD: move */
  class VariableSet : public std::set<Variable> {
  };
	
  /** 
   * A simple factor class to use in a factor graph.
   *
   * We make it noncopyable so we enforce the fact that factors are
   * kept in pointer containers. To be safe, you should make them
   * immutable, i.e., practicing functional programming. However, this
   * conflicts with efficiency as well, esp. in the case of incomplete
   * QR factorization. A solution is still being sought.
   * 
   * A Factor is templated on a Config, for example VectorConfig is a configuration of
   * labeled vectors. This way, we can have factors that might be defined on discrete
   * variables, continuous ones, or a combination of both. It is up to the config to 
   * provide the appropriate values at the appropriate time.
   */ 
  template <class Config>
  class Factor : boost::noncopyable, public Testable< Factor<Config> >
  {
  public:

    virtual ~Factor() {};
		
    /**
     * negative log probability 
     */
    virtual double error(const Config& c) const = 0;

    /**
     * return keys in preferred order
     */
    virtual std::list<std::string> keys() const = 0;
		
    /** 
     * @return the number of nodes the factor connects
     */
    virtual std::size_t size() const = 0;
  };
}

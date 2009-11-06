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
#include <map>
#include <list>
#include <boost/utility.hpp> // for noncopyable
#include "Testable.h"

namespace gtsam {

	/** A map from key to dimension, useful in various contexts */
  typedef std::map<std::string,int> Dimensions;
	
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

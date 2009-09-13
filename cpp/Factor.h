/**
 * @file    Factor.h
 * @brief   A simple factor class to use in a factor graph
 * @brief   factor
 * @author  Kai Ni
 */

// \callgraph

#pragma once

#include <set>
#include <list>
#include "FGConfig.h"

namespace gtsam {

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
   */ 
  class Factor : boost::noncopyable
  {
  public:

    virtual ~Factor() {};
		
    /**
     * negative log probability 
     */
    virtual double error(const FGConfig& c) const = 0;

    /**
     * print
     * @param s optional string naming the factor
     */
    virtual void print(const std::string& s="") const = 0;

    /**
     * equality up to tolerance
     * tricky to implement, see NonLinearFactor1 for an example
     */
    virtual bool equals(const Factor& f, double tol=1e-9) const = 0;

    virtual std::string dump() const = 0;

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

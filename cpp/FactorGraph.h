/**
 * @file    FactorGraph.h
 * @brief   Factor Graph Base Class
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <list>
#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "Factor.h"
#include "FGConfig.h"

namespace gtsam {
	
  /**
   * A factor graph is a bipartite graph with factor nodes connected to variable nodes.
   * In this class, however, only factor nodes are kept around.
   */
  template<class T> class FactorGraph
  {
  public:
    typedef typename boost::shared_ptr<T> shared_factor;
    typedef typename std::vector<shared_factor>::iterator iterator;
    typedef typename std::vector<shared_factor>::const_iterator const_iterator;

  protected:
    /** Collection of factors */
    std::vector<shared_factor> factors;
    std::map<std::string, std::list<int> > node_to_factors_;

  public:

    /** get the factors to a specific node */
      const std::list<int>& get_factors_to_node(const std::string& key) const {
      return node_to_factors_[key];
    }

    /** STL like, return the iterator pointing to the first factor */
    const_iterator begin() const {
      return factors.begin();
    }

    /** STL like, return the iterator pointing to the last factor */
    const_iterator end() const {
      return factors.end();
    }

    /** clear the factor graph */
    void clear(){
      factors.clear();
      node_to_factors_.clear();
    }

    /** Get a specific factor by index */
    shared_factor operator[](size_t i) const {
      return factors[i];
    }

    /** return the numbers of the factors in the factor graph */
    inline size_t size() const { return factors.size(); }

    /** Add a factor */
    void push_back(shared_factor ptr_f) {factors.push_back(ptr_f);}
		
    /** unnormalized error */
    double error(const FGConfig& c) const {
      double total_error = 0.;
      /** iterate over all the factors to accumulate the log probabilities */
      for(const_iterator factor=factors.begin(); factor!=factors.end(); factor++)
        total_error += (*factor)->error(c);

      return total_error;
    }

    /** Unnormalized probability. O(n) */
    double probPrime(const FGConfig& c) const {
      return exp(-0.5*error(c));
    }  

    /** print out graph */
    void print(const std::string& s = "FactorGraph") const{
      std::cout << s << std::endl;
      printf("size: %d\n", (int)size());
      for(const_iterator factor=factors.begin(); factor!=factors.end(); factor++)
        (*factor)->print();
    }

    /** Check equality */
    bool equals(const FactorGraph& fg, double tol=1e-9) const
    {
      /** check whether the two factor graphs have the same number of factors */
      if( factors.size() != fg.size() ) goto fail;
		
      /** check whether the factors are the same */
      for(size_t i=0;i<factors.size();i++)
        if ( ! factors[i]->equals(*fg.factors[i], tol) ) goto fail; //TODO: Doesn't this force order of factor insertion?
      return true;

    fail:
      print();
      fg.print();
      return false;
    }
  };

} // namespace gtsam

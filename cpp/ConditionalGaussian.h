/**
 * @file    ConditionalGaussian.h
 * @brief   Conditional Gaussian Base class
 * @author  Christian Potthast
 */


// \callgraph

#pragma once

#include <map>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include "Matrix.h"
#include "VectorConfig.h"
#include "Ordering.h"
#include "Testable.h"

namespace gtsam {
	
  /**
   * A conditional Gaussian functions as the node in a Bayes network
   * It has a set of parents y,z, etc. and implements a probability density on x.
   * The negative log-probability is given by || Rx - (d - Sy - Tz - ...)||^2
   */
  class ConditionalGaussian : boost::noncopyable, public Testable<ConditionalGaussian>
  {
  public:
    typedef std::map<std::string, Matrix>::const_iterator const_iterator;
		
  protected:

    /** the triangular matrix (square root information matrix) */
    Matrix R_; 

    /** the names and the matrices connecting to parent nodes */
    std::map<std::string, Matrix> parents_; 

    /** the RHS vector */
    Vector d_;

  public:
    typedef boost::shared_ptr<ConditionalGaussian> shared_ptr;

    /** constructor */
    ConditionalGaussian() {};
    
    /** Copy Constructor */
    ConditionalGaussian(const ConditionalGaussian &cg) :
      boost::noncopyable(), R_(cg.R_), parents_(cg.parents_), d_(cg.d_){}
		
    /** constructor with no parents
     * |Rx-d|
     */
    ConditionalGaussian(Vector d,
			Matrix R);
			
    /** constructor with only one parent
     * |Rx+Sy-d|
     */
    ConditionalGaussian(Vector d,
			Matrix R,
			const std::string& name1,
			Matrix S
      );
			
    /** constructor with two parents
     * |Rx+Sy+Tz-d|
     */
    ConditionalGaussian(Vector d,
			Matrix R,
			const std::string& name1,
			Matrix S,
			const std::string& name2,
			Matrix T
      );

    /**
     * constructor with number of arbitrary parents
     * |Rx+sum(Ai*xi)-d|
     */
    ConditionalGaussian(const Vector& d,
    		const Matrix& R,
    		const std::map<std::string, Matrix>& parents);

    /** deconstructor */
    virtual ~ConditionalGaussian() {};

    /** print */
    void print(const std::string& = "ConditionalGaussian") const;

    /** equals function */
    bool equals(const ConditionalGaussian &cg, double tol=1e-9) const;

    /** dimension of multivariate variable */
    size_t dim() const {return R_.size2();}

    /** return stuff contained in ConditionalGaussian */
    const Vector& get_d() const {return d_;}
    const Matrix& get_R() const {return R_;}

    /** STL like, return the iterator pointing to the first node */
    const_iterator const parentsBegin() const { return parents_.begin(); }

    /** STL like, return the iterator pointing to the last node */
    const_iterator const parentsEnd() const { return parents_.end(); }

    /** find the number of parents */
    size_t size() const {return parents_.size();}

    /** determine whether a key is among the parents */
    size_t contains(const std::string& key) const {return parents_.count(key);}

    /**
     * solve a conditional Gaussian
     * @param x configuration in which the parents values (y,z,...) are known
     * @return solution x = R \ (d - Sy - Tz - ...)
     */
    virtual Vector solve(const VectorConfig& x) const;

    /**
     * adds a parent
     */    
    void add(const std::string key, Matrix S){ parents_.insert(make_pair(key, S)); }
			
  private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(R_);
			ar & BOOST_SERIALIZATION_NVP(d_);
			ar & BOOST_SERIALIZATION_NVP(parents_);
		}
	};
}

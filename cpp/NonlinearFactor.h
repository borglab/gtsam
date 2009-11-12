/**
 * @file    NonlinearFactor.h
 * @brief   Non-linear factor class
 * @author  Kai Ni
 * @author  Carlos Nieto
 * @author  Christian Potthast
 * @author  Frank Dellaert
 */


// \callgraph

#pragma once

#include <list>

#include <boost/shared_ptr.hpp>
#include <boost/serialization/list.hpp>

#include "Factor.h"
#include "Vector.h"
#include "Matrix.h"
#include "GaussianFactor.h"

/**
 * Base Class
 * Just has the measurement and noise model
 */ 

namespace gtsam {

	// forward declaration of GaussianFactor
	//class GaussianFactor;
	//typedef boost::shared_ptr<GaussianFactor> shared_ptr;

  /**
   * Nonlinear factor which assumes Gaussian noise on a measurement
   * predicted by a non-linear function h.
   * 
   * Templated on a configuration type. The configurations are typically more general
	 * than just vectors, e.g., Rot3 or Pose3, which are objects in non-linear manifolds.
   */
	template <class Config>
  class NonlinearFactor : public Factor<Config>
  {
  protected:

    Vector z_;     // measurement
    double sigma_; // noise standard deviation
    std::list<std::string> keys_; // keys
		
  public:

    /** Default constructor, with easily identifiable bogus values */
    NonlinearFactor():z_(Vector_(2,888.0,999.0)),sigma_(0.1234567) {}

    /**
     *  Constructor
     *  @param z the measurement
     *  @param sigma the standard deviation
     */
    NonlinearFactor(const Vector& z, const double sigma) {
    	z_ = z;
    	sigma_ = sigma;
    }

    /** print */
    void print(const std::string& s = "") const {
    	std::cout << "NonlinearFactor " << s << std::endl;
    	gtsam::print(z_, "  z = ");
    	std::cout << "  sigma = " << sigma_ << std::endl;
    }

    /** Check if two NonlinearFactor objects are equal */
    bool equals(const Factor<Config>& f, double tol=1e-9) const {
    	const NonlinearFactor<Config>* p = dynamic_cast<const NonlinearFactor<Config>*> (&f);
    	if (p == NULL) return false;
      return equal_with_abs_tol(z_,p->z_,tol) && fabs(sigma_ - p->sigma_)<=tol;
    }

    /** Vector of errors */
    virtual Vector error_vector(const Config& c) const = 0;

    /** linearize to a GaussianFactor */
    virtual boost::shared_ptr<GaussianFactor> linearize(const Config& c) const = 0;

    /** get functions */
    double sigma() const {return sigma_;}
    Vector measurement() const {return z_;}
    std::list<std::string> keys() const {return keys_;}

    /** calculate the error of the factor */
    double error(const Config& c) const {
      Vector e = error_vector(c) / sigma_;
      return 0.5 * inner_prod(trans(e),e);
    };
		
    /** get the size of the factor */
    std::size_t size() const{return keys_.size();}

  private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(z_);
			ar & BOOST_SERIALIZATION_NVP(sigma_);
			ar & BOOST_SERIALIZATION_NVP(keys_);
		}

  }; // NonlinearFactor


  /**
   * a Gaussian nonlinear factor that takes 1 parameter
   * Note: cannot be serialized as contains function pointers
   * Specialized derived classes could do this
  */
  class NonlinearFactor1 : public NonlinearFactor<VectorConfig> {
  private:

		std::string key_;

  public:

		Vector (*h_)(const Vector&);
		Matrix (*H_)(const Vector&);

    /** Constructor */
    NonlinearFactor1(const Vector& z,		  // measurement
		     const double sigma,	  // variance
		     Vector (*h)(const Vector&),  // measurement function
		     const std::string& key1,     // key of the variable
		     Matrix (*H)(const Vector&)); // derivative of the measurement function

    void print(const std::string& s = "") const;

    /** Check if two factors are equal */
    bool equals(const Factor<VectorConfig>& f, double tol=1e-9) const;

    /** error function */
    inline Vector error_vector(const VectorConfig& c) const {
      return z_ - h_(c[key_]);
    }

    /** linearize a non-linearFactor1 to get a linearFactor1 */
    boost::shared_ptr<GaussianFactor> linearize(const VectorConfig& c) const;
  };

	/**
	 * a Gaussian nonlinear factor that takes 2 parameters
	 * Note: cannot be serialized as contains function pointers
	 * Specialized derived classes could do this
	*/
  class NonlinearFactor2 : public NonlinearFactor<VectorConfig> {

  private:

    std::string key1_;
    std::string key2_;

  public:

  	Vector (*h_)(const Vector&, const Vector&);
    Matrix (*H1_)(const Vector&, const Vector&);
    Matrix (*H2_)(const Vector&, const Vector&);

    /** Constructor */
    NonlinearFactor2(const Vector& z,	               // the measurement
		     const double sigma,	                       // the variance
		     Vector (*h)(const Vector&, const Vector&),  // the measurement function
		     const std::string& key1,                    // key of the first variable
		     Matrix (*H1)(const Vector&, const Vector&), // derivative of h in first variable
		     const std::string& key2,                    // key of the second variable
		     Matrix (*H2)(const Vector&, const Vector&));// derivative of h in second variable
			
    /** Print */
    void print(const std::string& s = "") const;

    /** Check if two factors are equal */
    bool equals(const Factor<VectorConfig>& f, double tol=1e-9) const;

    /** error function */
    inline Vector error_vector(const VectorConfig& c) const {
      return z_ - h_(c[key1_], c[key2_]); 
    }

    /** Linearize a non-linearFactor2 to get a linearFactor2 */
    boost::shared_ptr<GaussianFactor> linearize(const VectorConfig& c) const;
  };

  /* ************************************************************************* */
}

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
#include "Matrix.h"
#include "LinearFactor.h"

/**
 * Base Class
 * Just has the measurement and noise model
 */ 

namespace gtsam {

	// forward declaration of LinearFactor
	//class LinearFactor;
	//typedef boost::shared_ptr<LinearFactor> shared_ptr;

  /**
   * Nonlinear factor which assumes Gaussian noise on a measurement
   * predicted by a non-linear function h.
   */
  class NonlinearFactor : public Factor
  {
  protected:

    Vector z_;     // measurement
    double sigma_; // noise standard deviation
    std::list<std::string> keys_; // keys
		
  public:

    /** Default constructor, with easily identifiable bogus values */
    NonlinearFactor():z_(Vector_(2,888.0,999.0)),sigma_(0.1234567) {}

    /** Constructor */
    NonlinearFactor(const Vector& z, // the measurement
		    const double sigma);         // the variance

    /** Vector of errors */
    virtual Vector error_vector(const FGConfig& c) const = 0;

    /** linearize to a LinearFactor */
    virtual boost::shared_ptr<LinearFactor> linearize(const FGConfig& c) const = 0;

    /** get functions */
    double get_sigma() const {return sigma_;}
    Vector get_measurement() const {return z_;}
    std::list<std::string> get_keys() const {return keys_;}
    void set_keys(std::list<std::string> keys) {keys_ = keys;}

    /** calculate the error of the factor */
    double error(const FGConfig& c) const {
      Vector e = error_vector(c) / sigma_;
      return 0.5 * inner_prod(trans(e),e);
    };
		
    /** get the size of the factor */
    std::size_t size() const{return keys_.size();}

    /** Check if two NonlinearFactor objects are equal */
    bool equals(const NonlinearFactor& other, double tol=1e-9) const {
      return equal_with_abs_tol(z_,other.z_,tol) && fabs(sigma_ - other.sigma_)<=tol;
    }

    /** dump the information of the factor into a string **/
    std::string dump() const{return "";}

  private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			//  		ar & boost::serialization::base_object<Factor>(*this); // TODO: needed ?
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
  class NonlinearFactor1 : public NonlinearFactor {
  public:

    /** Constructor */
    NonlinearFactor1(const Vector& z,		  // measurement
		     const double sigma,	  // variance
		     Vector (*h)(const Vector&),  // measurement function
		     const std::string& key1,     // key of the variable
		     Matrix (*H)(const Vector&)); // derivative of the measurement function

    void print(const std::string& s = "") const;

    Vector (*h_)(const Vector&);
    std::string key1_;
    Matrix (*H_)(const Vector&);

    /** error function */
    inline Vector error_vector(const FGConfig& c) const { 
      return z_ - h_(c[key1_]);
    }

    /** linearize a non-linearFactor1 to get a linearFactor1 */
    boost::shared_ptr<LinearFactor> linearize(const FGConfig& c) const;

    /** Check if two factors are equal */
    bool equals(const Factor& f, double tol=1e-9) const;

    std::string dump() const {return "";}
  };

	/**
	 * a Gaussian nonlinear factor that takes 2 parameters
	 * Note: cannot be serialized as contains function pointers
	 * Specialized derived classes could do this
	*/
  class NonlinearFactor2 : public NonlinearFactor {
  public:

    /** Constructor */
    NonlinearFactor2(const Vector& z,	                         // the measurement
		     const double sigma,	                 // the variance
		     Vector (*h)(const Vector&, const Vector&),  // the measurement function
		     const std::string& key1,                    // key of the first variable
		     Matrix (*H1)(const Vector&, const Vector&), // derivative of h in first variable
		     const std::string& key2,                    // key of the second variable
		     Matrix (*H2)(const Vector&, const Vector&));// derivative of h in second variable
			
    void print(const std::string& s = "") const;

    Vector (*h_)(const Vector&, const Vector&);
    std::string key1_;
    Matrix (*H1_)(const Vector&, const Vector&);
    std::string key2_;
    Matrix (*H2_)(const Vector&, const Vector&);
		
    /** error function */
    inline Vector error_vector(const FGConfig& c) const { 
      return z_ - h_(c[key1_], c[key2_]); 
    }

    /** Linearize a non-linearFactor2 to get a linearFactor2 */
    boost::shared_ptr<LinearFactor> linearize(const FGConfig& c) const;

    /** Check if two factors are equal */
    bool equals(const Factor& f, double tol=1e-9) const;

    std::string dump() const{return "";};
  };

  /* ************************************************************************* */
}

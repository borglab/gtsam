/**
 * @file    GaussianConditional.h
 * @brief   Conditional Gaussian Base class
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <map>
#include <list>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "Conditional.h"
#include "VectorConfig.h"
#include "Matrix.h"

namespace gtsam {

class Ordering;

/**
 * A conditional Gaussian functions as the node in a Bayes network
 * It has a set of parents y,z, etc. and implements a probability density on x.
 * The negative log-probability is given by || Rx - (d - Sy - Tz - ...)||^2
 */
class GaussianConditional : public Conditional {

public:
	typedef std::map<std::string, Matrix> Parents;
	typedef Parents::const_iterator const_iterator;
	typedef boost::shared_ptr<GaussianConditional> shared_ptr;

protected:

	/** the triangular matrix (square root information matrix) - unit normalized */
	Matrix R_;

	/** the names and the matrices connecting to parent nodes */
	Parents parents_;

	/** the RHS vector */
	Vector d_;

	/** vector of standard deviations */
	Vector sigmas_;

public:

	/** default constructor needed for serialization */
	GaussianConditional(){}

	/** constructor */
	GaussianConditional(const std::string& key) :
		Conditional (key) {}

	/** constructor with no parents
	 * |Rx-d|
	 */
	GaussianConditional(const std::string& key, Vector d, Matrix R, Vector sigmas);

	/** constructor with only one parent
	 * |Rx+Sy-d|
	 */
	GaussianConditional(const std::string& key, Vector d, Matrix R,
			const std::string& name1, Matrix S, Vector sigmas);

	/** constructor with two parents
	 * |Rx+Sy+Tz-d|
	 */
	GaussianConditional(const std::string& key, Vector d, Matrix R,
			const std::string& name1, Matrix S, const std::string& name2, Matrix T, Vector sigmas);

	/**
	 * constructor with number of arbitrary parents
	 * |Rx+sum(Ai*xi)-d|
	 */
	GaussianConditional(const std::string& key, const Vector& d,
			const Matrix& R, const Parents& parents, Vector sigmas);

	/** deconstructor */
	virtual ~GaussianConditional() {}

	/** print */
	void print(const std::string& = "GaussianConditional") const;

	/** equals function */
	bool equals(const Conditional &cg, double tol = 1e-9) const;

	/** dimension of multivariate variable */
	size_t dim() const { return R_.size2();}

	/** return all parents */
	std::list<std::string> parents() const;

	/** return stuff contained in GaussianConditional */
	const Vector& get_d() const {return d_;}
	const Matrix& get_R() const {return R_;}
	const Vector& get_sigmas() const {return sigmas_;}

	/** STL like, return the iterator pointing to the first node */
	const_iterator const parentsBegin() const {
		return parents_.begin();
	}

	/** STL like, return the iterator pointing to the last node */
	const_iterator const parentsEnd() const {
		return parents_.end();
	}

	/** find the number of parents */
	size_t nrParents() const {
		return parents_.size();
	}

	/** determine whether a key is among the parents */
	size_t contains(const std::string& key) const {
		return parents_.count(key);
	}
	/**
	 * solve a conditional Gaussian
	 * @param x configuration in which the parents values (y,z,...) are known
	 * @return solution x = R \ (d - Sy - Tz - ...)
	 */
	virtual Vector solve(const VectorConfig& x) const;

	/**
	 * adds a parent
	 */
	void add(const std::string key, Matrix S){
		parents_.insert(make_pair(key, S));
	}

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("Conditional", boost::serialization::base_object<Conditional>(*this));
		ar & BOOST_SERIALIZATION_NVP(R_);
		ar & BOOST_SERIALIZATION_NVP(parents_);
		ar & BOOST_SERIALIZATION_NVP(d_);
		ar & BOOST_SERIALIZATION_NVP(sigmas_);
	}
};
}

/*
 * EqualityFactor.h
 *
 *  Created on: Aug 10, 2009
 *      Author: alexgc
 */

#ifndef EQUALITYFACTOR_H_
#define EQUALITYFACTOR_H_

#include "Factor.h"
#include "DeltaFunction.h"

namespace gtsam {

class EqualityFactor: public Factor {
public:
	typedef boost::shared_ptr<EqualityFactor> shared_ptr;


protected:
	Vector value_; /// forces a variable be equal to this value
	std::string key_; /// name of variable factor is attached to

public:
	/**
	 * Default constructor
	 */
	EqualityFactor();

	/**
	 * Constructor with initializiation
	 * @param constraint the value that the variable node is defined as equal to
	 * @param key identifier for the variable node
	 */
	EqualityFactor(const Vector& constraint, const std::string& key);

	/**
	 * Default Destructor
	 */
	~EqualityFactor() {}

	/**
	 * negative log probability
	 */
    double error(const FGConfig& c) const;

    /**
     * print
     * @param s optional string naming the factor
     */
    void print(const std::string& s="") const;

    /**
     * equality up to tolerance
     */
    bool equals(const Factor& f, double tol=1e-9) const;
    bool equals(const EqualityFactor& f, double tol=1e-9) const;

    /**
     * linearize
     */
    EqualityFactor::shared_ptr linearize() const;

    /**
     * returns a version of the factor as a string
     */
    std::string dump() const;

    // get functions
    std::string get_key() const {return key_;}
    Vector get_value() const {return value_;}

    /**
     * return keys in preferred order
     */
    std::list<std::string> keys() const;

    /**
     * @return the number of nodes the factor connects
     */
    std::size_t size() const {return 1;}

    /**
     * Returns the corresponding delta function for elimination
     */
    DeltaFunction::shared_ptr getDeltaFunction() const;
};

/** assert equals for testing - prints when not equal */
bool assert_equal(const EqualityFactor& actual, const EqualityFactor& expected, double tol=1e-9);

}

#endif /* EQUALITYFACTOR_H_ */

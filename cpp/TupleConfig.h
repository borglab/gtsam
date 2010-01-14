/*
 * TupleConfig.h
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts and Manohar Paluri
 */

#include <iostream>

#include "LieConfig-inl.h"

#pragma once

namespace gtsam {

  /**
   * PairConfig:  a config that holds two data types.
   */
  template<class J1, class X1, class J2, class X2>
  class PairConfig : public Testable<PairConfig<J1, X1, J2, X2> > {
  public:
//    typedef J1 Key1;
//    typedef J2 Key2;
//    typedef X1 Value1;
//    typedef X2 Value2;
    typedef LieConfig<J1, X1> Config1;
    typedef LieConfig<J2, X2> Config2;

  private:
    LieConfig<J1, X1> config1_;
    LieConfig<J2, X2> config2_;
    size_t size_;
    size_t dim_;

    PairConfig(const LieConfig<J1,X1>& config1, const LieConfig<J2,X2>& config2) :
      config1_(config1), config2_(config2),
      size_(config1.size()+config2.size()), dim_(gtsam::dim(config1)+gtsam::dim(config2)) {}

  public:

    /**
     * Default constructor creates an empty config.
     */
    PairConfig(): size_(0), dim_(0) {}

    /**
     * Copy constructor
     */
    PairConfig(const PairConfig<J1, X1, J2, X2>& c):
      config1_(c.config1_), config2_(c.config2_), size_(c.size_), dim_(c.dim_) {}

    /**
     * Print
     */
    void print(const std::string& s = "") const {
      std::cout << "TupleConfig " << s << ", size " << size_ << "\n";
      config1_.print(s + "Config1: ");
      config2_.print(s + "Config2: ");
    }

    /**
     * Test for equality in keys and values
     */
    bool equals(const PairConfig<J1, X1, J2, X2>& c, double tol=1e-9) const {
      return config1_.equals(c.config1_, tol) && config2_.equals(c.config2_, tol); }

    /**
     * operator[] syntax to get a value by j, throws invalid_argument if
     * value with specified j is not present.  Will generate compile-time
     * errors if j type does not match that on which the Config is templated.
     */
    const X1& operator[](const J1& j) const { return config1_[j]; }
    const X2& operator[](const J2& j) const { return config2_[j]; }

    /**
     * size is the total number of variables in this config.
     */
    size_t size() const { return size_; }

    /**
     * dim is the dimensionality of the tangent space
     */
    size_t dim() const { return dim_; }

  private:
    template<class Config, class Key, class Value>
    void insert_helper(Config& config, const Key& j, const Value& value) {
      config.insert(j, value);
      size_ ++;
      dim_ += gtsam::dim(value);
    }

    template<class Config, class Key>
    void erase_helper(Config& config, const Key& j) {
      size_t dim;
      config.erase(j, dim);
      dim_ -= dim;
      size_ --;
    }

  public:
    /**
     * expmap each element
     */
    PairConfig<J1,X1,J2,X2> expmap(const VectorConfig& delta) {
      return PairConfig(gtsam::expmap(config1_, delta), gtsam::expmap(config2_, delta)); }

    /**
     * Insert a variable with the given j
     */
    void insert(const J1& j, const X1& value) { insert_helper(config1_, j, value); }
    void insert(const J2& j, const X2& value) { insert_helper(config2_, j, value); }

    /**
     * Remove the variable with the given j.  Throws invalid_argument if the
     * j is not present in the config.
     */
    void erase(const J1& j) { erase_helper(config1_, j); }
    void erase(const J2& j) { erase_helper(config2_, j); }

    /**
     * Check if a variable exists
     */
    bool exists(const J1& j) const { return config1_.exists(j); }
    bool exists(const J2& j) const { return config2_.exists(j); }


  };

  template<class J1, class X1, class J2, class X2>
  inline PairConfig<J1,X1,J2,X2> expmap(PairConfig<J1,X1,J2,X2> c, const VectorConfig& delta) { return c.expmap(delta); }


}

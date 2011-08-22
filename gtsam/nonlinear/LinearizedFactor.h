/*
 * @file LinearizedFactor.h
 * @brief A dummy factor that allows a linear factor to act as a nonlinear factor
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/LinearApproxFactor-inl.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/Matrix.h>

#include <vector>
#include <iostream>
#include <boost/foreach.hpp>

namespace gtsam{

/**
 * Wrapper around a LinearApproxFactor with some extra interfaces
 */
template <class VALUES, class KEY>
class LinearizedFactor : public LinearApproxFactor<VALUES, KEY> {

public:
  /** base type */
  typedef LinearApproxFactor<VALUES, KEY> Base;

  /** shared pointer for convenience */
  typedef boost::shared_ptr<LinearizedFactor<VALUES,KEY> > shared_ptr;

  /** decoder for keys - avoids the use of a full ordering */
  typedef std::map<Index, KEY> KeyLookup;

protected:
  /** default constructor for serialization */
  LinearizedFactor() {}

public:

  /**
   * Use this constructor when starting with linear keys and adding in a label
   * @param label is a label to add to the keys
   * @param lin_factor is a gaussian factor with linear keys (no labels baked in)
   * @param values is assumed to have the correct key structure with labels
   */
  LinearizedFactor(JacobianFactor::shared_ptr lin_factor,
      const KeyLookup& decoder, const VALUES& lin_points)
  : Base(lin_factor->get_model()) {
    this->b_ = lin_factor->getb();
    BOOST_FOREACH(const Index& idx, *lin_factor) {
      // find nonlinear multirobot key
      typename KeyLookup::const_iterator decode_it = decoder.find(idx);
      assert(decode_it != decoder.end());
      KEY key = decode_it->second;

      // extract linearization point
      assert(lin_points.exists(key));
      typename KEY::Value value = lin_points[key];
      this->lin_points_.insert(key, value);  // NOTE: will not overwrite

      // extract Jacobian
      Matrix A = lin_factor->getA(lin_factor->find(idx));
      this->matrices_.insert(std::make_pair(key, A));

      // store keys
      this->nonlinearKeys_.push_back(key);
      this->keys_.push_back(key);
    }
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("LinearApproxFactor",
        boost::serialization::base_object<Base>(*this));
  }
};


} // \namespace gtsam

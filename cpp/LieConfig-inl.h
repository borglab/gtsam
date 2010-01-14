/*
 * LieConfig.cpp
 *
 *  Created on: Jan 8, 2010
 *      Author: richard
 */

#include "LieConfig.h"

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <utility>
#include <iostream>
#include <stdexcept>

#include "VectorConfig.h"

using namespace std;

namespace gtsam {

  template<class J, class T>
  void LieConfig<J,T>::print(const string &s) const {
       cout << "LieConfig " << s << ", size " << values_.size() << "\n";
       BOOST_FOREACH(const typename Values::value_type& v, values_)
         gtsam::print(v.second, (string)(v.first));
     }

  template<class J, class T>
  bool LieConfig<J,T>::equals(const LieConfig<J,T>& expected, double tol) const {
    if (values_.size() != expected.values_.size()) return false;
    BOOST_FOREACH(const typename Values::value_type& v, values_) {
    	if (!exists(v.first)) return false;
      if(!gtsam::equal(v.second, expected[v.first], tol))
        return false;
    }
    return true;
  }

  template<class J, class T>
  const T& LieConfig<J,T>::at(const J& j) const {
    const_iterator it = values_.find(j);
    if (it == values_.end()) throw std::invalid_argument("invalid j: " + (string)j);
    else return it->second;
  }

  template<class J, class T>
  void LieConfig<J,T>::insert(const J& name, const T& val) {
    values_.insert(make_pair(name, val));
    dim_ += dim(val);
  }

  template<class J, class T>
  void LieConfig<J,T>::erase(const J& j) {
    size_t dim; // unused
    erase(j, dim);
  }

  template<class J, class T>
  void LieConfig<J,T>::erase(const J& j, size_t& dim) {
    iterator it = values_.find(j);
    if (it == values_.end()) throw std::invalid_argument("invalid j: " + (string)j);
    dim = gtsam::dim(it->second);
    dim_ -= dim;
    values_.erase(it);
  }

  // todo: insert for every element is inefficient
  template<class J, class T>
  LieConfig<J,T> expmap(const LieConfig<J,T>& c, const VectorConfig& delta) {
		LieConfig<J,T> newConfig;
		typedef pair<J,T> Value;
		BOOST_FOREACH(const Value& value, c) {
			const J& j = value.first;
			const T& pj = value.second;
			string jstr = (string)j;
			if (delta.contains(jstr)) {
				const Vector& dj = delta[jstr];
				newConfig.insert(j, expmap(pj,dj));
			} else
			newConfig.insert(j, pj);
		}
		return newConfig;
	}

  // todo: insert for every element is inefficient
  template<class J, class T>
  LieConfig<J,T> expmap(const LieConfig<J,T>& c, const Vector& delta) {
    if(delta.size() != dim(c))
      throw invalid_argument("Delta vector length does not match config dimensionality.");
    LieConfig<J,T> newConfig;
    int delta_offset = 0;
		typedef pair<J,T> Value;
		BOOST_FOREACH(const Value& value, c) {
			const J& j = value.first;
			const T& pj = value.second;
      int cur_dim = dim(pj);
      newConfig.insert(j,expmap(pj,sub(delta, delta_offset, delta_offset+cur_dim)));
      delta_offset += cur_dim;
    }
    return newConfig;
  }
}



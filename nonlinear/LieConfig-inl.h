/**
 * @file LieConfig.cpp
 * @author Richard Roberts
 */

#pragma once

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <utility>
#include <iostream>
#include <stdexcept>

#include <gtsam/linear/VectorConfig.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/Lie-inl.h>

#include <gtsam/nonlinear/LieConfig.h>

#define INSTANTIATE_LIE_CONFIG(J) \
  /*INSTANTIATE_LIE(T);*/ \
  /*template LieConfig<J> expmap(const LieConfig<J>&, const VectorConfig&);*/ \
  /*template LieConfig<J> expmap(const LieConfig<J>&, const Vector&);*/ \
  /*template VectorConfig logmap(const LieConfig<J>&, const LieConfig<J>&);*/ \
  template class LieConfig<J>;

using namespace std;

namespace gtsam {

/* ************************************************************************* */
  template<class J>
  void LieConfig<J>::print(const string &s) const {
       cout << "LieConfig " << s << ", size " << values_.size() << "\n";
       BOOST_FOREACH(const KeyValuePair& v, values_) {
         gtsam::print(v.second, (string)(v.first));
       }
     }

  /* ************************************************************************* */
  template<class J>
  bool LieConfig<J>::equals(const LieConfig<J>& expected, double tol) const {
    if (values_.size() != expected.values_.size()) return false;
    BOOST_FOREACH(const KeyValuePair& v, values_) {
    	if (!expected.exists(v.first)) return false;
      if(!gtsam::equal(v.second, expected[v.first], tol))
        return false;
    }
    return true;
  }

  /* ************************************************************************* */
  template<class J>
  const typename J::Value_t& LieConfig<J>::at(const J& j) const {
    const_iterator it = values_.find(j);
    if (it == values_.end()) throw std::invalid_argument("invalid j: " + (string)j);
    else return it->second;
  }

  /* ************************************************************************* */
  template<class J>
  size_t LieConfig<J>::dim() const {
  	size_t n = 0;
  	BOOST_FOREACH(const KeyValuePair& value, values_)
  		n += value.second.dim();
  	return n;
  }

  /* ************************************************************************* */
  template<class J>
  VectorConfig LieConfig<J>::zero() const {
  	VectorConfig z;
  	BOOST_FOREACH(const KeyValuePair& value, values_)
  		z.insert(value.first,gtsam::zero(value.second.dim()));
  	return z;
  }

  /* ************************************************************************* */
  template<class J>
  void LieConfig<J>::insert(const J& name, const typename J::Value_t& val) {
    values_.insert(make_pair(name, val));
  }

  /* ************************************************************************* */
  template<class J>
  void LieConfig<J>::insert(const LieConfig<J>& cfg) {
	  BOOST_FOREACH(const KeyValuePair& v, cfg.values_)
		 insert(v.first, v.second);
  }

  /* ************************************************************************* */
  template<class J>
  void LieConfig<J>::update(const LieConfig<J>& cfg) {
	  BOOST_FOREACH(const KeyValuePair& v, values_) {
	  	boost::optional<typename J::Value_t> t = cfg.exists_(v.first);
	  	if (t) values_[v.first] = *t;
	  }
  }

  /* ************************************************************************* */
  template<class J>
  void LieConfig<J>::update(const J& j, const typename J::Value_t& val) {
	  	values_[j] = val;
  }

  /* ************************************************************************* */
  template<class J>
  std::list<J> LieConfig<J>::keys() const {
	  std::list<J> ret;
	  BOOST_FOREACH(const KeyValuePair& v, values_)
		  ret.push_back(v.first);
	  return ret;
  }

  /* ************************************************************************* */
  template<class J>
  void LieConfig<J>::erase(const J& j) {
    size_t dim; // unused
    erase(j, dim);
  }

  /* ************************************************************************* */
  template<class J>
  void LieConfig<J>::erase(const J& j, size_t& dim) {
    iterator it = values_.find(j);
    if (it == values_.end()) throw std::invalid_argument("invalid j: " + (string)j);
    dim = it->second.dim();
    values_.erase(it);
  }

  /* ************************************************************************* */
  // todo: insert for every element is inefficient
  template<class J>
  LieConfig<J> LieConfig<J>::expmap(const VectorConfig& delta) const {
		LieConfig<J> newConfig;
		typedef pair<J,typename J::Value_t> KeyValue;
		BOOST_FOREACH(const KeyValue& value, this->values_) {
			const J& j = value.first;
			const typename J::Value_t& pj = value.second;
			Symbol jkey = (Symbol)j;
			if (delta.contains(jkey)) {
				const Vector& dj = delta[jkey];
				newConfig.insert(j, pj.expmap(dj));
			} else
				newConfig.insert(j, pj);
		}
		return newConfig;
	}

  /* ************************************************************************* */
  // todo: insert for every element is inefficient
  template<class J>
  LieConfig<J> LieConfig<J>::expmap(const Vector& delta) const {
    if(delta.size() != dim()) {
    	cout << "LieConfig::dim (" << dim() << ") <> delta.size (" << delta.size() << ")" << endl;
      throw invalid_argument("Delta vector length does not match config dimensionality.");
    }
    LieConfig<J> newConfig;
    int delta_offset = 0;
		typedef pair<J,typename J::Value_t> KeyValue;
		BOOST_FOREACH(const KeyValue& value, this->values_) {
			const J& j = value.first;
			const typename J::Value_t& pj = value.second;
      int cur_dim = pj.dim();
      newConfig.insert(j,pj.expmap(sub(delta, delta_offset, delta_offset+cur_dim)));
      delta_offset += cur_dim;
    }
    return newConfig;
  }

  /* ************************************************************************* */
  // todo: insert for every element is inefficient
  // todo: currently only logmaps elements in both configs
  template<class J>
  VectorConfig LieConfig<J>::logmap(const LieConfig<J>& cp) const {
  	VectorConfig delta;
		typedef pair<J,typename J::Value_t> KeyValue;
  	BOOST_FOREACH(const KeyValue& value, cp) {
  		if(this->exists(value.first))
  			delta.insert(value.first, this->at(value.first).logmap(value.second));
  	}
  	return delta;
  }

}



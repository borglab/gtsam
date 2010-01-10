/*
 * LieConfig.cpp
 *
 *  Created on: Jan 8, 2010
 *      Author: richard
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include "LieConfig.h"

using namespace std;

#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

  template<class T>
  void LieConfig<T>::print(const string &s) const {
       cout << "LieConfig " << s << ", size " << values_.size() << "\n";
       pair<string, T> v;
       BOOST_FOREACH(v, values_)
         gtsam::print(v.second, v.first + ": ");
     }

  template<class T>
  bool LieConfig<T>::equals(const LieConfig<T>& expected, double tol) const {
    if (values_.size() != expected.values_.size()) return false;
    pair<string, T> v;
    BOOST_FOREACH(v, values_) {
      boost::optional<const T&> expval = expected.gettry(v.first);
      if(!expval || !gtsam::equal(v.second, *expval, tol))
        return false;
    }
    return true;
  }

  template<class T>
  LieConfig<T> expmap(const LieConfig<T>& c, const VectorConfig& delta) {
    LieConfig<T> newConfig;
    string j; T pj;
    FOREACH_PAIR(j, pj, c.values_) {
        if (delta.contains(j)) {
            const Vector& dj = delta[j];
            //check_size(j,vj,dj);
            newConfig.insert(j, expmap(pj,dj));
        } else
            newConfig.insert(j, pj);
    }
    return newConfig;
  }

  template<class T>
  LieConfig<T> expmap(const LieConfig<T>& c, const Vector& delta) {
    LieConfig<T> newConfig;
    pair<string, Vector> value;
    int delta_offset = 0;
    BOOST_FOREACH(value, c) {
      int cur_dim = dim(value.second);
      newConfig.insert(value.first,
          expmap(value.second,
          sub(delta, delta_offset, delta_offset+cur_dim)));
      delta_offset += cur_dim;
    }
    return newConfig;
  }

}

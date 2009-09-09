/**
 * @file    FGConfig.cpp
 * @brief   Factor Graph Configuration
 * @brief   fgConfig
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include "FGConfig.h"

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
void check_size(const string& key, const Vector & vj, const Vector & dj) {
  if (dj.size()!=vj.size()) {
    cout << "For key \"" << key << "\"" << endl;
    cout << "vj.size = " << vj.size() << endl;
    cout << "dj.size = " << dj.size() << endl;
    throw(std::invalid_argument("FGConfig::+ mismatched dimensions"));
  }
}

/* ************************************************************************* */
FGConfig FGConfig::exmap(const FGConfig & delta) const
{
	FGConfig newConfig;
  for (const_iterator it = values.begin(); it!=values.end(); it++) {
    string j = it->first;
    const Vector &vj = it->second;
    const Vector& dj = delta[j];
    check_size(j,vj,dj);
    newConfig.insert(j, vj + dj);
  }
  return newConfig;
}

/* ************************************************************************* */
Vector FGConfig::get(const std::string& name) const {
  const_iterator it = values.find(name);
  if (it==values.end()) {
    print();
    cout << "asked for key " << name << endl;
    throw(std::invalid_argument("FGConfig::[] invalid key"));
  }
  return it->second;
}

/* ************************************************************************* */
void FGConfig::print(const std::string& name) const {
  odprintf("FGConfig %s\n", name.c_str());
  odprintf("size: %d\n", values.size());
  string j; Vector v;
  FOREACH_PAIR(j, v, values) {
    odprintf("%s:", j.c_str());
    gtsam::print(v);
  }
}

/* ************************************************************************* */
bool FGConfig::equals(const FGConfig& expected, double tol) const {
  string j; Vector vActual;
  if( values.size() != expected.size() ) goto fail;

  // iterate over all nodes
  FOREACH_PAIR(j, vActual, values) {
    Vector vExpected = expected[j];
    if(!equal_with_abs_tol(vExpected,vActual,tol)) 
      goto fail;
  }

  return true;

fail: 
  // print and return false
  print();
  expected.print();
  return false;
}

/* ************************************************************************* */


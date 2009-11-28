/**
 * @file    VectorConfig.cpp
 * @brief   Factor Graph Configuration
 * @brief   VectorConfig
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include "VectorConfig.h"

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
    throw(std::invalid_argument("VectorConfig::+ mismatched dimensions"));
  }
}

/* ************************************************************************* */
VectorConfig VectorConfig::scale(double gain) {
	VectorConfig scaled;
	string key; Vector val;
	FOREACH_PAIR(key, val, values) {
		scaled.insert(key, gain*val);
	}
	return scaled;
}

/* ************************************************************************* */
VectorConfig VectorConfig::exmap(const VectorConfig & delta) const
{
	VectorConfig newConfig;
	for (const_iterator it = values.begin(); it!=values.end(); it++) {
		string j = it->first;
		const Vector &vj = it->second;
		if (delta.contains(j)) {
			const Vector& dj = delta[j];
			check_size(j,vj,dj);
			newConfig.insert(j, vj + dj);
		} else {
			newConfig.insert(j, vj);
		}
	}
	return newConfig;
}

/* ************************************************************************* */
Vector VectorConfig::get(const std::string& name) const {
  const_iterator it = values.find(name);
  if (it==values.end()) {
    print();
    cout << "asked for key " << name << endl;
    throw(std::invalid_argument("VectorConfig::[] invalid key"));
  }
  return it->second;
}

/* ************************************************************************* */
void VectorConfig::print(const std::string& name) const {
  odprintf("VectorConfig %s\n", name.c_str());
  odprintf("size: %d\n", values.size());
  string j; Vector v;
  FOREACH_PAIR(j, v, values) {
    odprintf("%s:", j.c_str());
    gtsam::print(v);
  }
}

/* ************************************************************************* */
bool VectorConfig::equals(const VectorConfig& expected, double tol) const {
  string j; Vector vActual;
  if( values.size() != expected.size() ) return false;

  // iterate over all nodes
  FOREACH_PAIR(j, vActual, values) {
    Vector vExpected = expected[j];
    if(!equal_with_abs_tol(vExpected,vActual,tol)) 
    	return false;
  }
  return true;
}

/* ************************************************************************* */


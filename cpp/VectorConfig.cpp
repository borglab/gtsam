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

namespace gtsam {

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
std::vector<std::string> VectorConfig::get_names() const {
  std::vector<std::string> names;
  for(const_iterator it=values.begin(); it!=values.end(); it++)
    names.push_back(it->first);
  return names;
}

/* ************************************************************************* */
VectorConfig& VectorConfig::insert(const std::string& name, const Vector& val) {
  values.insert(std::make_pair(name,val));
  return *this;
}

/* ************************************************************************* */
void VectorConfig::add(const std::string& j, const Vector& a) {
	Vector& vj = values[j];
	if (vj.size()==0) vj = a; else vj += a;
}

/* ************************************************************************* */
size_t VectorConfig::dim() const {
	size_t result=0;
	string key; Vector v;
	FOREACH_PAIR(key, v, values) result += v.size();
	return result;
}

/* ************************************************************************* */
VectorConfig VectorConfig::scale(double s) const {
	VectorConfig scaled;
	string key; Vector val;
	FOREACH_PAIR(key, val, values)
		scaled.insert(key, s*val);
	return scaled;
}

/* ************************************************************************* */
VectorConfig VectorConfig::operator*(double s) const {
	return scale(s);
}

/* ************************************************************************* */
VectorConfig VectorConfig::operator-() const {
	VectorConfig result;
	string j; Vector v;
	FOREACH_PAIR(j, v, values)
		result.insert(j, -v);
	return result;
}

/* ************************************************************************* */
void VectorConfig::operator+=(const VectorConfig& b) {
	string j; Vector b_j;
	FOREACH_PAIR(j, b_j, b.values) {
		iterator it = values.find(j);
		if (it==values.end())
			insert(j, b_j);
		else
			it->second += b_j;
	}
}

/* ************************************************************************* */
VectorConfig VectorConfig::operator+(const VectorConfig& b) const {
	VectorConfig result = *this;
	result += b;
	return result;
}

/* ************************************************************************* */
VectorConfig VectorConfig::operator-(const VectorConfig& b) const {
	VectorConfig result;
	string j; Vector v;
	FOREACH_PAIR(j, v, values)
		result.insert(j, v - b.get(j));
	return result;
}

/* ************************************************************************* */
VectorConfig expmap(const VectorConfig& original, const VectorConfig& delta)
{
	VectorConfig newConfig;
	string j; Vector vj;
	FOREACH_PAIR(j, vj, original.values) {
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
VectorConfig expmap(const VectorConfig& original, const Vector& delta)
{
	VectorConfig newConfig;
	size_t i = 0;
	string j; Vector vj;
	FOREACH_PAIR(j, vj, original.values) {
		size_t mj = vj.size();
		Vector dj = sub(delta, i, i+mj);
		newConfig.insert(j, vj + dj);
		i += mj;
	}
	return newConfig;
}

/* ************************************************************************* */
const Vector& VectorConfig::get(const std::string& name) const {
  const_iterator it = values.find(name);
  if (it==values.end()) {
    print();
    cout << "asked for key " << name << endl;
    throw(std::invalid_argument("VectorConfig::[] invalid key"));
  }
  return it->second;
}

/* ************************************************************************* */
Vector& VectorConfig::getReference(const std::string& name) {
  iterator it = values.find(name);
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
double VectorConfig::dot(const VectorConfig& b) const {
	string j; Vector v; double result = 0.0;
	FOREACH_PAIR(j, v, values) result += gtsam::dot(v,b.get(j));
	return result;
}

/* ************************************************************************* */
double dot(const VectorConfig& a, const VectorConfig& b) {
	return a.dot(b);
}
/* ************************************************************************* */

} // gtsam

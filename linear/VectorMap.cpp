/**
 * @file    VectorMap.cpp
 * @brief   Factor Graph Configuration
 * @brief   VectorMap
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <gtsam/linear/VectorMap.h>

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void check_size(varid_t key, const Vector & vj, const Vector & dj) {
  if (dj.size()!=vj.size()) {
    cout << "For key \"" << key << "\"" << endl;
    cout << "vj.size = " << vj.size() << endl;
    cout << "dj.size = " << dj.size() << endl;
    throw(std::invalid_argument("VectorMap::+ mismatched dimensions"));
  }
}

/* ************************************************************************* */
std::vector<varid_t> VectorMap::get_names() const {
  std::vector<varid_t> names;
  for(const_iterator it=values.begin(); it!=values.end(); it++)
    names.push_back(it->first);
  return names;
}

/* ************************************************************************* */
VectorMap& VectorMap::insert(varid_t name, const Vector& v) {
  values.insert(std::make_pair(name,v));
  return *this;
}

/* ************************************************************************* */
VectorMap& VectorMap::insertAdd(varid_t j, const Vector& a) {
	Vector& vj = values[j];
	if (vj.size()==0) vj = a; else vj += a;
	return *this;
}

/* ************************************************************************* */
void VectorMap::insert(const VectorMap& config) {
	for (const_iterator it = config.begin(); it!=config.end(); it++)
		insert(it->first, it->second);
}

/* ************************************************************************* */
void VectorMap::insertAdd(const VectorMap& config) {
	for (const_iterator it = config.begin(); it!=config.end(); it++)
    insertAdd(it->first,it->second);
}

/* ************************************************************************* */
size_t VectorMap::dim() const {
	size_t result=0;
	for (const_iterator it = begin(); it != end(); it++)
		result += it->second.size();
	return result;
}

/* ************************************************************************* */
const Vector& VectorMap::operator[](varid_t name) const {
  return values.at(name);
}

/* ************************************************************************* */
Vector& VectorMap::operator[](varid_t name) {
  return values.at(name);
}

/* ************************************************************************* */
VectorMap VectorMap::scale(double s) const {
	VectorMap scaled;
	for (const_iterator it = begin(); it != end(); it++)
		scaled.insert(it->first, s*it->second);
	return scaled;
}

/* ************************************************************************* */
VectorMap VectorMap::operator*(double s) const {
	return scale(s);
}

/* ************************************************************************* */
VectorMap VectorMap::operator-() const {
	VectorMap result;
	for (const_iterator it = begin(); it != end(); it++)
		result.insert(it->first, - it->second);
	return result;
}

/* ************************************************************************* */
void VectorMap::operator+=(const VectorMap& b) {
	insertAdd(b);
}

/* ************************************************************************* */
VectorMap VectorMap::operator+(const VectorMap& b) const {
	VectorMap result = *this;
	result += b;
	return result;
}

/* ************************************************************************* */
VectorMap VectorMap::operator-(const VectorMap& b) const {
	VectorMap result;
	for (const_iterator it = begin(); it != end(); it++)
		result.insert(it->first, it->second - b[it->first]);
	return result;
}

/* ************************************************************************* */
VectorMap& VectorMap::zero() {
	for (iterator it = begin(); it != end(); it++)
		std::fill(it->second.begin(), it->second.end(), 0.0);
	return *this;
}

/* ************************************************************************* */
VectorMap VectorMap::zero(const VectorMap& x) {
	VectorMap cloned(x);
	return cloned.zero();
}

/* ************************************************************************* */
Vector VectorMap::vector() const {
	Vector result(dim());

	size_t cur_dim = 0;
	varid_t j; Vector vj;
	FOREACH_PAIR(j, vj, values) {
		subInsert(result, vj, cur_dim);
		cur_dim += vj.size();
	}
	return result;
}

/* ************************************************************************* */
VectorMap expmap(const VectorMap& original, const VectorMap& delta)
{
	VectorMap newConfig;
	varid_t j; Vector vj; // rtodo: copying vector?
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
VectorMap expmap(const VectorMap& original, const Vector& delta)
{
	VectorMap newConfig;
	size_t i = 0;
	varid_t j; Vector vj; // rtodo: copying vector?
	FOREACH_PAIR(j, vj, original.values) {
		size_t mj = vj.size();
		Vector dj = sub(delta, i, i+mj);
		newConfig.insert(j, vj + dj);
		i += mj;
	}
	return newConfig;
}

/* ************************************************************************* */
void VectorMap::print(const string& name) const {
  odprintf("VectorMap %s\n", name.c_str());
  odprintf("size: %d\n", values.size());
  for (const_iterator it = begin(); it != end(); it++) {
    odprintf("%d:", it->first);
    gtsam::print(it->second);
  }
}

/* ************************************************************************* */
bool VectorMap::equals(const VectorMap& expected, double tol) const {
  if( values.size() != expected.size() ) return false;

  // iterate over all nodes
  for (const_iterator it = begin(); it != end(); it++) {
    Vector vExpected = expected[it->first];
    if(!equal_with_abs_tol(vExpected,it->second,tol))
    	return false;
  }
  return true;
}

/* ************************************************************************* */
double VectorMap::dot(const VectorMap& b) const {
  double result = 0.0; // rtodo: copying vector
  for (const_iterator it = begin(); it != end(); it++)
  	result += gtsam::dot(it->second,b[it->first]);
	return result;
}

/* ************************************************************************* */
double dot(const VectorMap& a, const VectorMap& b) {
	return a.dot(b);
}

/* ************************************************************************* */
void scal(double alpha, VectorMap& x) {
	for (VectorMap::iterator xj = x.begin(); xj != x.end(); xj++)
		scal(alpha, xj->second);
}

/* ************************************************************************* */
void axpy(double alpha, const VectorMap& x, VectorMap& y) {
	VectorMap::const_iterator xj = x.begin();
	for (VectorMap::iterator yj = y.begin(); yj != y.end(); yj++, xj++)
		axpy(alpha, xj->second, yj->second);
}

/* ************************************************************************* */
void print(const VectorMap& v, const std::string& s){
	v.print(s);
}

/* ************************************************************************* */

} // gtsam

// MATLAB container proxies used by certifiable_mosek.i. The typedef names
// match the generator's names for these STL types.
namespace std {

#include <vector>
template<T>
class vector {
  vector();
  size_t size() const;
  T at(size_t pos) const;
  void push_back(const T& value);
};
typedef std::vector<double> vectordouble;

#include <map>
template<K, V>
class map {
  map();
  size_t size() const;
  double at(K key) const;
  void emplace(K key, double value);
};
typedef std::map<std::string, double> mapstringdouble;
typedef std::map<gtsam::Key, gtsam::DenseIndex> mapKeyDenseIndex;

}  // namespace std

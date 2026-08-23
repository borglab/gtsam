#pragma once

#include <cstddef>
#include <string>

namespace adapters {

using Index = std::size_t;

class BaseAdapter {
 public:
  BaseAdapter() = default;

  int inherited(int value, bool verbose = false) const {
    return value + (verbose ? 1 : 0);
  }
};

template <typename T>
class Adapter : public BaseAdapter {
 public:
  Adapter() = default;

  int exact(int value) { return value; }
  int exactConst(int value) const { return value; }
  static int exactStatic(int value) { return value; }

  int omittedDefault(int value, bool verbose = false) const {
    return value + (verbose ? 1 : 0);
  }

  int referenceArgument(int& value) const { return ++value; }

  int hiddenOverload(int value) const { return value; }
  int hiddenOverload(double value) const { return static_cast<int>(value); }

  int declaredOverload(int value) const { return value; }
  double declaredOverload(double value) const { return value; }

  const T& at(std::size_t) const { return value_; }
  const T& front() const { return value_; }

  int alias(Index index) const { return static_cast<int>(index); }

  static int staticOmitted(int value, bool verbose = false) {
    return value + (verbose ? 1 : 0);
  }

  template <typename U>
  U templated(U value, bool verbose = false) const {
    return value + (verbose ? U{1} : U{0});
  }

  const std::string& lambda(const std::string& value = "fallback",
                            bool verbose = false) const {
    text_ = value + (verbose ? "!" : "");
    return text_;
  }

 private:
  T value_{};
  mutable std::string text_;
};

inline int exactGlobal(int value) { return value; }

inline int globalOmitted(int value, bool verbose = false) {
  return value + (verbose ? 1 : 0);
}

inline int globalHidden(int value) { return value; }
inline int globalHidden(double value) { return static_cast<int>(value); }

inline int globalOverload(int value) { return value; }
inline double globalOverload(double value) { return value; }

template <typename T>
T globalTemplated(T value, bool verbose = false) {
  return value + (verbose ? T{1} : T{0});
}

}  // namespace adapters

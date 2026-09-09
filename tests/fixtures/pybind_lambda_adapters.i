#include <pybind_lambda_adapters.h>

namespace adapters {

class BaseAdapter {
  BaseAdapter();

  @pybind_lambda
  int inherited(int value) const;
};

template<T = {int}>
class Adapter: adapters::BaseAdapter {
  Adapter();

  int exact(int value);
  int exactConst(int value) const;
  static int exactStatic(int value);

  @pybind_lambda
  int omittedDefault(int value) const;

  @pybind_lambda
  int referenceArgument(int value) const;

  int hiddenOverload(int value) const;

  int declaredOverload(int value) const;
  double declaredOverload(double value) const;

  @pybind_lambda
  T at(size_t index) const;

  @pybind_lambda
  T front() const;

  @pybind_lambda
  int alias(int index) const;

  @pybind_lambda
  static int staticOmitted(int value);

  template<U = {double}>
  U templated(U value) const;

  @pybind_lambda
  const string& lambda(const string& value = "fallback") const;
};

int exactGlobal(int value);

@pybind_lambda
int globalOmitted(int value);

int globalHidden(int value);

int globalOverload(int value);
double globalOverload(double value);

template<T = {int}>
T globalTemplated(T value);

}

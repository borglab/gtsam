// Test for templated constructor
class TemplatedConstructor {
  TemplatedConstructor();

  template<T={string, int, double}>
  TemplatedConstructor(const T& arg);
};

// Test for a scoped value inside a template
template <T = {Result}>
class ScopedTemplate {
  // T should be properly substituted here.
  ScopedTemplate(const T::Value& arg);
};


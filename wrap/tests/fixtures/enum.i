enum Kind { Dog, Cat };

namespace gtsam {
enum VerbosityLM {
  SILENT,
  SUMMARY,
  TERMINATION,
  LAMBDA,
  TRYLAMBDA,
  TRYCONFIG,
  DAMPED,
  TRYDELTA
};

class Pet {
  enum Kind { Dog, Cat };

  Pet(const string &name, Kind type);

  string name;
  Kind type;
};
}  // namespace gtsam

namespace gtsam {
class NumericContainers {
  NumericContainers();
  bool solve(const std::map<std::string, double>& params = std::map<std::string, double>());
  std::vector<double> evrs() const;
  const std::map<gtsam::Key, gtsam::DenseIndex>& dims() const;
  std::vector<double> echo(std::vector<double> values) const;
  std::optional<std::pair<std::vector<double>, std::map<double, double>>> pair() const;
  std::optional<std::vector<double>> optional(std::optional<std::vector<double>> values) const;
};
}

class UnaryFactor: public NoiseModelFactor1<Pose2> {
  double mx_, my_; ///< X and Y measurements

public:
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  Vector evaluateError(const Pose2& q,
                       boost::optional<Matrix&> H = boost::none) const
  {
    const double cosTheta = cos(q.theta());
    const double sinTheta = sin(q.theta());
    if (H) (*H) = (gtsam::Matrix(2, 3) <<
            cosTheta, -sinTheta, 0.0, 
            sinTheta, cosTheta, 0.0).finished();
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }
};

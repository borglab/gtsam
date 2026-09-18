namespace gtsam {

template<T>
class SerializableTypedefFixture {
  SerializableTypedefFixture();
};

@serializable
typedef gtsam::SerializableTypedefFixture<int> SerializableFixture;
typedef gtsam::SerializableTypedefFixture<double> PlainFixture;

}  // namespace gtsam

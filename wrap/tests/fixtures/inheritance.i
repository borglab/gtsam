// A base class
virtual class MyBase {
};

// A templated class
template<T = {gtsam::Point2, gtsam::Matrix, A}>
virtual class MyTemplate : MyBase {
  MyTemplate();

  template<ARG = {gtsam::Point2, gtsam::Point3, gtsam::Vector, gtsam::Matrix}>
  ARG templatedMethod(const ARG& t);

  // Stress test templates and pointer combinations
  void accept_T(const T& value) const;
  void accept_Tptr(T* value) const;
  T* return_Tptr(T* value) const;
  T  return_T(T@ value) const;
  pair<T*,T*> create_ptrs () const;
  pair<T ,T*> create_MixedPtrs () const;
  pair<T*,T*> return_ptrs (T* p1, T* p2) const;

  static This Level(const T& K);
};


virtual class ForwardKinematicsFactor : gtsam::BetweenFactor<gtsam::Pose3> {};

template <T = {double}>
virtual class ParentHasTemplate : MyTemplate<T> {};

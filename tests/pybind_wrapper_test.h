#pragma once

#include <iostream>
#include <memory>
#include <string>

namespace anzu {

class PointBase {
 public:
  virtual double sum() const = 0;
  virtual ~PointBase() {}
};

namespace sub {

class Point2 : public PointBase {
 public:
  explicit Point2(double x, double y = 10.0) : x_{x}, y_{y} {}
  double x() const { return x_; }
  double y() const { return y_; }
  double sum() const override;
  double func_with_default_args(double a, double b = 20.0) const {
    return a + b;
  }
  void print(const std::string& s) const { std::cout << s << std::endl; }

 private:
  double x_, y_;
};

}  // namespace sub

class Point3 : public PointBase {
 public:
  Point3(double x, double y, double z) : x_{x}, y_{y}, z_{z} {}
  double x() const { return x_; }
  // Overload method.
  double x(double to_add) const { return x_ + to_add; }
  double y() const { return y_; }
  double z() const { return z_; }
  double sum() const override;

 private:
  double x_, y_, z_;
};

template <class POINT>
class Template {
 public:
  explicit Template(const POINT& point, double a = 10) : point_{point} {}
  Template(const Template<POINT>& other) : point_{other.point_} {}

  double overload() const { return point_.sum() + point_.x(); }
  double overload(const POINT& point) const {
    return point_.sum() + point.sum();
  }
  double overload(const Template<POINT>& other) const {
    return point_.sum() + other.overload();
  }

  POINT point() const { return point_; }

  POINT method_on_template_type(const POINT& point) const { return point; }
  Template<POINT> method_on_this(const POINT& point) const { return *this; }

  static Template<POINT> static_method(const Template<POINT>& other,
                                       double dummy) {
    return other.method_on_this(other.point());
  }

  template <typename OTHER_POINT>
  double template_method(const OTHER_POINT& other) const {
    return point_.x() + other.x();
  }

 private:
  POINT point_;
};

template <class T1, class T2>
class Template2 {
 public:
  Template2(const T1& t1, const T2& t2) : t1_(t1), t2_(t2) {}

  double sum_x() const { return t1_.x() + t2_.x(); }

  double sum_x(const T1& other1) const {
    return t1_.x() + t2_.x() + other1.x();
  }

  double sum_x(const std::shared_ptr<T2>& other2) const {
    return t1_.x() + t2_.x() + other2->x();
  }

  double sum_x(const T1& other1, const std::shared_ptr<T2>& other2) const {
    return t1_.x() + t2_.x() + other1.x() + other2->x();
  }

 private:
  T1 t1_;
  T2 t2_;

 public:
  T1 property_t1{10};
};

class Ignore {
 public:
  explicit Ignore(int x) {}
};

namespace sub2 {
class Point4 {
 public:
  Point4(const sub::Point2& p_in, double z_in, double w_in)
      : p(p_in), z(z_in), w(w_in) {}
  double sum() { return p.sum() + z + w; }

  const anzu::sub::Point2 p;
  double z;
  double w;
};
}  // namespace sub2

// A function on the base class.
double global_func_on_base(const std::shared_ptr<PointBase>& point);

}  // namespace anzu

// Overload functions.
double global_func_overloads(const std::shared_ptr<anzu::sub::Point2>& point2);
double global_func_overloads(const std::shared_ptr<anzu::Point3>& point3);

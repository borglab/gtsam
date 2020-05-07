#include <gtsam/geometry/Line3.h>

namespace gtsam {

Line3 Line3::retract(const Vector4 &v, OptionalJacobian<4, 4> H) const {
    Vector3 w;
    w << v[0], v[1], 0;
    Rot3 eps;
    if (H) {
        Eigen::Matrix3d Dw_mat = Eigen::Matrix3d::Zero();
        OptionalJacobian<3, 3> Dw(Dw_mat);
        Dw->setZero();
        eps = Rot3::Expmap(w, Dw);
        H->block<2, 2>(0, 0) = Dw->block<2, 2>(0, 0);
        (*H)(2, 2) = 1;
        (*H)(3, 3) = 1;
    } else {
        eps = Rot3::Expmap(w);
    }
    Rot3 Rt = R_ * eps;
    return Line3(Rt, a_ + v[2], b_ + v[3]);
}

Vector4 Line3::localCoordinates(const Line3 &q, OptionalJacobian<4, 4> H) const {
    Vector3 local_rot;
    Vector4 local;
    if (H) {
        Eigen::Matrix3d Dw_mat = Eigen::Matrix3d::Zero();
        OptionalJacobian<3, 3> Dw(Dw_mat);
        Dw->setZero();
        local_rot = Rot3::Logmap(R_.inverse() * q.R_, Dw);
        H->block<2, 2>(0, 0) = Dw->block<2, 2>(0, 0);
        (*H)(2, 2) = 1;
        (*H)(3, 3) = 1;
    } else {
        local_rot = Rot3::Logmap(R_.inverse() * q.R_);
    }
    local << local_rot[0], local_rot[1], q.a_ - a_, q.b_ - b_;
    return local;
}

void Line3::print(const std::string &s) const {
    std::cout << s << std::endl;
    R_.print("R:\n");
    std::cout << "a: " << a_ << ", b: " << b_ << std::endl;
}

bool Line3::equals(const Line3 &l2, double tol) const {
    Vector4 diff = localCoordinates(l2);
    return fabs(diff[0]) < tol && fabs(diff[1]) < tol
        && fabs(diff[2]) < tol && fabs(diff[3]) < tol;
}

Unit3 Line3::project(OptionalJacobian<2, 4> Dline) const {
    Vector3 V_0;
    V_0 << -b_, a_, 0.0;

    Unit3 l;
    if (Dline) {
        // Jacobian of the normalized Unit3 projected line with respect to
        // un-normalized Vector3 projected line in homogeneous coordinates.
        Eigen::Matrix<double, 2, 3> D_mat = Eigen::Matrix<double, 2, 3>::Zero();
        OptionalJacobian<2, 3> D_unit_line(D_mat);
        l = Unit3::FromPoint3(Point3(R_ * V_0), D_unit_line);
        // Jacobian of the un-normalized Vector3 line with respect to
        // input 3D line
        Eigen::Matrix<double, 3, 4> D_vec_line = Eigen::Matrix<double, 3, 4>::Zero();
        D_vec_line.col(0) = a_ * R_.r3();
        D_vec_line.col(1) = b_ * R_.r3();
        D_vec_line.col(2) = R_.r2();
        D_vec_line.col(3) = -R_.r1();
        // Jacobian of output wrt input is the product of the two.
        Eigen::Matrix<double, 2, 4> Dline_mat = (*D_unit_line) * D_vec_line;
        Dline = OptionalJacobian<2, 4>(Dline_mat);
    } else {
        l = Unit3::FromPoint3(Point3(R_ * V_0));
    }
    return l;
}

Point3 Line3::point(double distance) const {
    // defining "center" of the line to be the point where it
    // intersects rotated XY axis
    Point3 center(a_, b_, 0);
    Point3 rotated_center = R_ * center;
    return rotated_center + distance * R_.r3();
}

Line3 transformTo(const Pose3 &wTc, const Line3 &wL,
                  OptionalJacobian<4, 6> Dpose, OptionalJacobian<4, 4> Dline) {
    Rot3 wRc = wTc.rotation();
    Rot3 cRw = wRc.inverse();
    Rot3 cRl = cRw * wL.R_;

    Vector2 w_ab;
    Vector3 t = ((wL.R_).transpose() * wTc.translation());
    Vector2 c_ab(wL.a_ - t[0], wL.b_ - t[1]);

    if (Dpose) {
        // translation due to translation
        Matrix3 cRl_mat = cRl.matrix();
        Matrix3 lRc = cRl_mat.transpose();
        Dpose->block<1, 3>(2, 3) = -lRc.row(0);
        Dpose->block<1, 3>(3, 3) = -lRc.row(1);

        Dpose->block<1, 3>(0, 0) = -lRc.row(0);
        Dpose->block<1, 3>(1, 0) = -lRc.row(1);
    }
    if (Dline) {
        Dline->col(0) << 1.0, 0.0, 0.0, -t[2];
        Dline->col(1) << 0.0, 1.0, t[2], 0.0;
        Dline->col(2) << 0.0, 0.0, 1.0, 0.0;
        Dline->col(3) << 0.0, 0.0, 0.0, 1.0;
    }
    return Line3(cRl, c_ab[0], c_ab[1]);
}

}

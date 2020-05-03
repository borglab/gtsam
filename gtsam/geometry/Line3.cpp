#include <gtsam/geometry/Line3.h>

namespace gtsam {


Vector6 Line3::points()
{
    Vector3 mid;
    mid << a_, b_, 0;
    Vector3 mid_rot = R_*mid;
    Vector3 start = mid_rot + R_.r3();
    Vector3 end = mid_rot - R_.r3();
    Vector6 start_end;
    start_end << start(0), start(1), start(2),
                    end(0), end(1), end(2);
    return start_end;
}


Line3 Line3::retract(const Vector4 &v, OptionalJacobian<4,4> H) const {
    Vector3 w;
    w << v[0], v[1], 0;
    Rot3 eps;
    if(H)
    {
        OptionalJacobian<3,3> Dw;
        eps = Rot3::Expmap(w, Dw);
        H->block<2,2>(0,0) = Dw->block<2,2>(0,0);
        (*H)(2,2) = 1;
        (*H)(3,3) = 1;
    }
    else
    {
        eps = Rot3::Expmap(w);
    }
    Rot3 Rt = R_*eps;
    return Line3(Rt, a_+v[2], b_+v[3]);
}


Vector4 Line3::localCoordinates(const Line3 &q, OptionalJacobian<4,4> H) const {
    Vector3 local_rot;
    Vector4 local;
    Vector2 ab_q(q.V());
    if(H)
    {
        OptionalJacobian<3,3> Dw;
        local_rot = Rot3::Logmap(R_.inverse()*q.R(), Dw);
        H->block<2,2>(0, 0) = Dw->block<2,2>(0,0);
        (*H)(2,2) = 1;
        (*H)(3,3) = 1;
    }
    else
    {
        local_rot = Rot3::Logmap(R_.inverse()*q.R());
    }
    local << local_rot[0], local_rot[1], ab_q[0] - a_, ab_q[1] - b_;
    return local;
}


void Line3::print(const std::string &s) const
{
    std::cout << s << std::endl;
    R_.print("R:\n");
    std::cout << "a: " << a_ << ", b: " << b_ << std::endl;
}


bool Line3::equals(const Line3 &l2, double tol) const
{
    Vector4 diff = localCoordinates(l2);
    return fabs(diff[0]) < tol && fabs(diff[1]) < tol
           && fabs(diff[2]) < tol && fabs(diff[3]) < tol;
}


Point3 Line3::project(OptionalJacobian<3, 4> Dline) const
{
    Vector3 V_0;
    V_0 << -b_, a_, 0;

    Point3 l = R_*V_0;
    if(Dline)
    {
        Dline->setZero();
        Dline->col(0) = a_*R_.r3();
        Dline->col(1) = b_*R_.r3();
        Dline->col(2) = R_.r2();
        Dline->col(3) = -R_.r1();
    }
    return l;
}


Point3 projectLine(const Line3 &L,
        OptionalJacobian<3, 4> Dline)
{
    return L.project(Dline);
}


Line3 transformTo(const Pose3 &p, const Line3 &l,
                    OptionalJacobian<4, 6> Dpose, OptionalJacobian<4, 4> Dline)
{
    Rot3 w_R_l = l.R();
    Rot3 w_R_c = p.rotation();
    Rot3 c_R_w = w_R_c.inverse();
    Rot3 c_R_l = c_R_w*w_R_l;

    Vector2 ab_w = l.V();
    Vector3 t = (w_R_l.transpose()*p.translation());
    Vector2 ab_c(ab_w[0] - t[0], ab_w[1] - t[1]);

    if(Dpose)
    {
        // translation due to translation
        Matrix3 c_R_l_mat = c_R_l.matrix();
        Matrix3 l_R_c = c_R_l_mat.transpose();
        Dpose->block<1,3>(2, 3) = -l_R_c.row(0);
        Dpose->block<1,3>(3, 3) = -l_R_c.row(1);

        Dpose->block<1,3>(0, 0) = -l_R_c.row(0);
        Dpose->block<1,3>(1, 0) = -l_R_c.row(1);
    }
    if(Dline)
    {
        Dline->col(0) << 1.0, 0.0, 0.0, -t[2];
        Dline->col(1) << 0.0, 1.0, t[2], 0.0;
        Dline->col(2) << 0.0, 0.0, 1.0, 0.0;
        Dline->col(3) << 0.0, 0.0, 0.0, 1.0;
    }
    return Line3(c_R_l, ab_c[0], ab_c[1]);
}

}

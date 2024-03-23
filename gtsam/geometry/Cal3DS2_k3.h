/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2_k3.h
 * @brief Calibration of a camera with radial distortion whose coefficients are not only k1, k2 but also k3,
 * calculations in base class Cal3DS2_Base_k3
 * @date Dec 19, 2023
 * @author demul
 * @author Hyeonjin Jeong
 */

#pragma once

#include <gtsam/geometry/Cal3DS2_k3_Base.h>

namespace gtsam
{
/**
 * @brief Calibration of a camera with radial distortion whose coefficients are not only k1, k2 but also k3
 * Lie-group behaviors for optimization.
 * \sa Cal3DS2_Base_k3
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3DS2_k3 : public Cal3DS2_k3_Base
{
    using Base = Cal3DS2_k3_Base;

   public:
    enum
    {
        dimension = 10
    };

    /// @name Standard Constructors
    /// @{

    /// Default Constructor with only unit focal length
    Cal3DS2_k3() = default;

    Cal3DS2_k3(
        double fx,
        double fy,
        double s,
        double u0,
        double v0,
        double k1,
        double k2,
        double p1 = 0.0,
        double p2 = 0.0,
        double k3 = 0.0,
        double tol =
            1e-5)  // The order in which the coefficients are sorted follows the same order as the one used in opencv
        : Base(fx, fy, s, u0, v0, k1, k2, p1, p2, k3, tol)
    {
    }

    ~Cal3DS2_k3() override
    {
    }

    /// @}
    /// @name Advanced Constructors
    /// @{

    Cal3DS2_k3(const Vector10& v) : Base(v)
    {
    }

    /// @}
    /// @name Testable
    /// @{

    /// Output stream operator
    GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const Cal3DS2_k3& cal);

    /// print with optional string
    void print(const std::string& s = "") const override;

    /// assert equality up to a tolerance
    bool equals(const Cal3DS2_k3& K, double tol = 10e-9) const;

    /// @}
    /// @name Manifold
    /// @{

    /// Given delta vector, update calibration
    Cal3DS2_k3 retract(const Vector& d) const;

    /// Given a different calibration, calculate update to obtain it
    Vector localCoordinates(const Cal3DS2_k3& T2) const;

    /// Return dimensions of calibration manifold object
    size_t dim() const override
    {
        return Dim();
    }

    /// Return dimensions of calibration manifold object
    inline static size_t Dim()
    {
        return dimension;
    }

    /// @}
    /// @name Clone
    /// @{

    /// @return a deep copy of this object
    boost::shared_ptr<Base> clone() const override
    {
        return boost::shared_ptr<Base>(new Cal3DS2_k3(*this));
    }

    /// @}

   private:
    /// @name Advanced Interface
    /// @{

    /** Serialization function */
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/)
    {
        ar& boost::serialization::make_nvp("Cal3DS2_k3", boost::serialization::base_object<Cal3DS2_k3_Base>(*this));
    }

    /// @}
};

template <>
struct traits<Cal3DS2_k3> : public internal::Manifold<Cal3DS2_k3>
{
};

template <>
struct traits<const Cal3DS2_k3> : public internal::Manifold<Cal3DS2_k3>
{
};
}  // namespace gtsam

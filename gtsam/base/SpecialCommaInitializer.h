/**
 * @file    SpecialCommaInitializer.h
 * @brief   A special comma initializer for Eigen that is implicitly convertible to Vector and Matrix.
 * @author  Richard Roberts
 * @created Oct 10, 2013
 */

#pragma once

#include <gtsam/3rdparty/gtsam_eigen_includes.h>

namespace Eigen {
  namespace internal {
    // Row-vectors not tested
    //template<typename XprType>
    //inline void resizeHelper(XprType& xpr, DenseIndex sizeIncrement,
    //    typename boost::enable_if_c<
    //    XprType::ColsAtCompileTime == Dynamic && XprType::RowsAtCompileTime == 1>::type* = 0)
    //{
    //  xpr.conservativeResize(xpr.cols() + sizeIncrement);
    //}

    template<typename XprType>
    inline void resizeHelper(XprType& xpr, typename XprType::Index sizeIncrement,
        typename boost::enable_if_c<
        XprType::RowsAtCompileTime == Dynamic && XprType::ColsAtCompileTime == 1>::type* = 0)
    {
      xpr.conservativeResize(xpr.rows() + sizeIncrement);
    }

    template<typename XprType>
    inline void resizeHelper(XprType& xpr, typename XprType::Index sizeIncrement,
        typename boost::enable_if_c<
        XprType::ColsAtCompileTime == Dynamic>::type* = 0)
    {
      assert(false);
    }
  }

  /// A special comma initializer for Eigen that is implicitly convertible to Vector and Matrix.
  template<typename XprType>
  class SpecialCommaInitializer :
      public CommaInitializer<XprType>,
      public MatrixBase<SpecialCommaInitializer<XprType> >
  {
  private:
    bool dynamic_;

  public:
    typedef MatrixBase<SpecialCommaInitializer<XprType> > Base;
    typedef CommaInitializer<XprType> CommaBase;

    EIGEN_DENSE_PUBLIC_INTERFACE(SpecialCommaInitializer)
    typedef typename internal::conditional<internal::must_nest_by_value<XprType>::ret,
        XprType, const XprType&>::type ExpressionTypeNested;
    typedef typename XprType::InnerIterator InnerIterator;

    // Forward to base class
    inline SpecialCommaInitializer(XprType& xpr, const typename XprType::Scalar& s, bool dynamic) :
          CommaBase(xpr, s), dynamic_(dynamic) {}

    // Forward to base class
    template<typename OtherDerived>
    inline SpecialCommaInitializer(XprType& xpr, const DenseBase<OtherDerived>& other, bool dynamic) :
    CommaBase(xpr, other), dynamic_(dynamic) {}

    inline Index rows() const { return CommaBase::m_xpr.rows(); }
    inline Index cols() const { return CommaBase::m_xpr.cols(); }
    inline Index outerStride() const { return CommaBase::m_xpr.outerStride(); }
    inline Index innerStride() const { return CommaBase::m_xpr.innerStride(); }

    inline CoeffReturnType coeff(Index row, Index col) const
    {
      return CommaBase::m_xpr.coeff(row, col);
    }

    inline CoeffReturnType coeff(Index index) const
    {
      return CommaBase::m_xpr.coeff(index);
    }

    inline const Scalar& coeffRef(Index row, Index col) const
    {
      return CommaBase::m_xpr.const_cast_derived().coeffRef(row, col);
    }

    inline const Scalar& coeffRef(Index index) const
    {
      return CommaBase::m_xpr.const_cast_derived().coeffRef(index);
    }

    inline Scalar& coeffRef(Index row, Index col)
    {
      return CommaBase::m_xpr.const_cast_derived().coeffRef(row, col);
    }

    inline Scalar& coeffRef(Index index)
    {
      return CommaBase::m_xpr.const_cast_derived().coeffRef(index);
    }

    template<int LoadMode>
    inline const PacketScalar packet(Index row, Index col) const
    {
      return CommaBase::m_xpr.template packet<LoadMode>(row, col);
    }

    template<int LoadMode>
    inline void writePacket(Index row, Index col, const PacketScalar& x)
    {
      CommaBase::m_xpr.const_cast_derived().template writePacket<LoadMode>(row, col, x);
    }

    template<int LoadMode>
    inline const PacketScalar packet(Index index) const
    {
      return CommaBase::m_xpr.template packet<LoadMode>(index);
    }

    template<int LoadMode>
    inline void writePacket(Index index, const PacketScalar& x)
    {
      CommaBase::m_xpr.const_cast_derived().template writePacket<LoadMode>(index, x);
    }

    const XprType& _expression() const { return CommaBase::m_xpr; }

    /// Override base class comma operators to return this class instead of the base class.
    SpecialCommaInitializer& operator,(const typename XprType::Scalar& s)
    {
      // If dynamic, resize the underlying object
      if(dynamic_)
      {
        // Dynamic expansion currently only tested for column-vectors
        assert(XprType::RowsAtCompileTime == Dynamic);
        // Current col should be zero and row should be at the end
        assert(CommaBase::m_col == 1);
        assert(CommaBase::m_row == CommaBase::m_xpr.rows() - CommaBase::m_currentBlockRows);
        resizeHelper(CommaBase::m_xpr, 1);
      }
      (void) CommaBase::operator,(s);
      return *this;
    }

    /// Override base class comma operators to return this class instead of the base class.
    template<typename OtherDerived>
    SpecialCommaInitializer& operator,(const DenseBase<OtherDerived>& other)
    {
      // If dynamic, resize the underlying object
      if(dynamic_)
      {
        // Dynamic expansion currently only tested for column-vectors
        assert(XprType::RowsAtCompileTime == Dynamic);
        // Current col should be zero and row should be at the end
        assert(CommaBase::m_col == 1);
        assert(CommaBase::m_row == CommaBase::m_xpr.rows() - CommaBase::m_currentBlockRows);
        resizeHelper(CommaBase::m_xpr, other.size());
      }
      (void) CommaBase::operator,(other);
      return *this;
    }
  };

  namespace internal {
    template<typename ExpressionType>
    struct traits<SpecialCommaInitializer<ExpressionType> > : traits<ExpressionType>
    {
    };
  }

}

namespace gtsam {
  class Vec
  {
    Eigen::VectorXd vector_;
    bool dynamic_;

  public:
    Vec(Eigen::VectorXd::Index size) : vector_(size), dynamic_(false) {}

    Vec() : dynamic_(true) {}

    Eigen::SpecialCommaInitializer<Eigen::VectorXd> operator<< (double s)
    {
      if(dynamic_)
        vector_.resize(1);
      return Eigen::SpecialCommaInitializer<Eigen::VectorXd>(vector_, s, dynamic_);
    }

    template<typename OtherDerived>
    Eigen::SpecialCommaInitializer<Eigen::VectorXd> operator<<(const Eigen::DenseBase<OtherDerived>& other)
    {
      if(dynamic_)
        vector_.resize(other.size());
      return Eigen::SpecialCommaInitializer<Eigen::VectorXd>(vector_, other, dynamic_);
    }
  };

  class Mat
  {
    Eigen::MatrixXd matrix_;

  public:
    Mat(Eigen::MatrixXd::Index rows, Eigen::MatrixXd::Index cols) : matrix_(rows, cols) {}

    Eigen::SpecialCommaInitializer<Eigen::MatrixXd> operator<< (double s)
    {
      return Eigen::SpecialCommaInitializer<Eigen::MatrixXd>(matrix_, s, false);
    }

    template<typename OtherDerived>
    Eigen::SpecialCommaInitializer<Eigen::MatrixXd> operator<<(const Eigen::DenseBase<OtherDerived>& other)
    {
      return Eigen::SpecialCommaInitializer<Eigen::MatrixXd>(matrix_, other, false);
    }
  };
}

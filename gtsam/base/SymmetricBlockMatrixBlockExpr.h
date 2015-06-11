/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file    SymmetricBlockMatrixBlockExpr.h
* @brief   Matrix expression for a block of a SymmetricBlockMatrix
* @author  Richard Roberts
* @date    Nov 20, 2013
*/
#pragma once

#include <gtsam/base/Matrix.h>

namespace gtsam { template<typename SymmetricBlockMatrixType> class SymmetricBlockMatrixBlockExpr; }
namespace gtsam { class SymmetricBlockMatrix; }

// traits class for Eigen expressions
namespace Eigen
{
  namespace internal
  {
    template<typename SymmetricBlockMatrixType>
    struct traits<gtsam::SymmetricBlockMatrixBlockExpr<SymmetricBlockMatrixType> > :
    public traits<typename gtsam::const_selector<
      SymmetricBlockMatrixType, gtsam::SymmetricBlockMatrix, gtsam::Matrix, const gtsam::Matrix>::type>
    {
    };
  }
}

namespace gtsam
{
  /// A matrix expression that references a single block of a SymmetricBlockMatrix.  Depending on
  /// the position of the block, this expression will behave either as a regular matrix block, a
  /// transposed matrix block, or a symmetric matrix block.  The only reason this class is templated
  /// on SymmetricBlockMatrixType is to allow for both const and non-const references.
  template<typename SymmetricBlockMatrixType>
  class SymmetricBlockMatrixBlockExpr : public Eigen::EigenBase<SymmetricBlockMatrixBlockExpr<SymmetricBlockMatrixType> >
  {
  protected:
    SymmetricBlockMatrixType& xpr_; ///< The referenced SymmetricBlockMatrix
    DenseIndex densei_; ///< The scalar indices of the referenced block
    DenseIndex densej_; ///< The scalar indices of the referenced block
    DenseIndex denseRows_; ///< The scalar size of the referenced block
    DenseIndex denseCols_; ///< The scalar size of the referenced block
    enum BlockType { Plain, SelfAdjoint, Transposed } blockType_; ///< The type of the referenced block, as determined by the block position
    typedef SymmetricBlockMatrixBlockExpr<SymmetricBlockMatrixType> This;

  public:
    // Typedefs and constants used in Eigen
    typedef typename const_selector<SymmetricBlockMatrixType, SymmetricBlockMatrix,
      typename Eigen::internal::traits<This>::Scalar&, typename Eigen::internal::traits<This>::Scalar>::type ScalarRef;
    typedef typename Eigen::internal::traits<This>::Scalar Scalar;
    typedef typename Eigen::internal::traits<This>::Index Index;
    static const Index ColsAtCompileTime = Eigen::Dynamic;
    static const Index RowsAtCompileTime = Eigen::Dynamic;

    typedef typename const_selector<SymmetricBlockMatrixType, SymmetricBlockMatrix, Matrix, const Matrix>::type
      DenseMatrixType;

    typedef Eigen::Map<DenseMatrixType, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> > OffDiagonal;
    typedef Eigen::SelfAdjointView<Eigen::Block<DenseMatrixType>, Eigen::Upper> SelfAdjointView;
    typedef Eigen::TriangularView<Eigen::Block<DenseMatrixType>, Eigen::Upper> TriangularView;

  protected:
    mutable Eigen::Block<DenseMatrixType> myBlock_;
    template<typename OtherSymmetricBlockMatrixType> friend class SymmetricBlockMatrixBlockExpr;

  public:
    /// Create a SymmetricBlockMatrixBlockExpr from the specified block of a SymmetricBlockMatrix.
    SymmetricBlockMatrixBlockExpr(SymmetricBlockMatrixType& blockMatrix, Index iBlock, Index jBlock) :
      xpr_(blockMatrix), myBlock_(blockMatrix.matrix_.block(0, 0, 0, 0))
    {
      initIndices(iBlock, jBlock);
    }

    /// Create a SymmetricBlockMatrixBlockExpr from the specified range of blocks of a
    /// SymmetricBlockMatrix.
    SymmetricBlockMatrixBlockExpr(SymmetricBlockMatrixType& blockMatrix,
      Index firstRowBlock, Index firstColBlock, Index rowBlocks, Index colBlocks) :
      xpr_(blockMatrix), myBlock_(blockMatrix.matrix_.block(0, 0, 0, 0))
    {
      initIndices(firstRowBlock, firstColBlock, rowBlocks, colBlocks);
    }

    /// Create a SymmetricBlockMatrixBlockExpr from the specified range of blocks of a
    /// SymmetricBlockMatrix.
    SymmetricBlockMatrixBlockExpr(SymmetricBlockMatrixType& blockMatrix, Index firstBlock, Index blocks, char /*dummy*/) :
      xpr_(blockMatrix), myBlock_(blockMatrix.matrix_.block(0, 0, 0, 0))
    {
      initIndices(firstBlock, firstBlock, blocks, blocks);
    }

    inline Index rows() const { return blockType_ != Transposed ? denseRows_ : denseCols_; }
    inline Index cols() const { return blockType_ != Transposed ? denseCols_ : denseRows_; }

    inline BlockType blockType() const { return blockType_;  }

    inline ScalarRef operator()(Index row, Index col) const
    {
      return coeffInternal<ScalarRef>(row, col);
    }

    inline OffDiagonal knownOffDiagonal() const
    {
      typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> DynamicStride;

      // We can return a Map if we are either on an off-diagonal block, or a block of size 0 or 1
      assert(blockType_ != SelfAdjoint || (denseRows_ <= 1 && denseCols_ <= 1));
      if(blockType_ == Transposed)
      {
        // Swap the inner and outer stride to produce a transposed Map
        Eigen::Block<DenseMatrixType> block = const_cast<This&>(*this).xpr_.matrix_.block(densei_, densej_, denseRows_, denseCols_);
        return Eigen::Map<DenseMatrixType, 0, DynamicStride>(block.data(), block.cols(), block.rows(),
          DynamicStride(block.innerStride(), block.outerStride()));
      }
      else
      {
        Eigen::Block<DenseMatrixType> block = const_cast<This&>(*this).xpr_.matrix_.block(densei_, densej_, denseRows_, denseCols_);
        return Eigen::Map<DenseMatrixType, 0, DynamicStride>(block.data(), block.rows(), block.cols(),
          DynamicStride(block.outerStride(), block.innerStride()));
      }
    }

    inline SelfAdjointView selfadjointView() const
    {
      assert(blockType_ == SelfAdjoint);
      return myBlock_;
    }

    inline TriangularView triangularView() const
    {
      assert(blockType_ == SelfAdjoint);
      return myBlock_;
    }

    template<typename Dest> inline void evalTo(Dest& dst) const
    {
      // Just try to assign to the object using either a selfadjoint view or a block view
      if(blockType_ == SelfAdjoint)
        dst = selfadjointView();
      else if(blockType_ == Plain)
        dst = myBlock_;
      else
        dst = myBlock_.transpose();
    }

    //template<typename MatrixType> inline void evalTo(const Eigen::SelfAdjointView<MatrixType, Eigen::Upper>& rhs) const
    //{
    //  if(blockType_ == SelfAdjoint)
    //    rhs.nestedExpression().triangularView<Eigen::Upper>() = triangularView();
    //  else
    //    throw std::invalid_argument("Cannot assign an off-diagonal block to a self-adjoint matrix");
    //}

    //template<typename MatrixType> inline void evalTo(const Eigen::TriangularView<MatrixType, Eigen::Upper>& rhs) const
    //{
    //  if(blockType_ == SelfAdjoint)
    //    rhs.nestedExpression().triangularView<Eigen::Upper>() = triangularView();
    //  else
    //    throw std::invalid_argument("Cannot assign an off-diagonal block to a self-adjoint matrix");
    //}

    template<typename RhsDerived>
    This& operator=(const Eigen::MatrixBase<RhsDerived>& rhs)
    {
      // Just try to assign to the object using either a selfadjoint view or a block view
      if(blockType_ == SelfAdjoint)
        triangularView() = rhs.derived().template triangularView<Eigen::Upper>();
      else if(blockType_ == Plain)
        myBlock_ = rhs.derived();
      else
        myBlock_.transpose() = rhs.derived();
      return *this;
    }

    template<typename MatrixType>
    This& operator=(const Eigen::SelfAdjointView<MatrixType, Eigen::Upper>& rhs)
    {
      if(blockType_ == SelfAdjoint)
        triangularView() = rhs.nestedExpression().template triangularView<Eigen::Upper>();
      else
        throw std::invalid_argument("Cannot assign a self-adjoint matrix to an off-diagonal block");
      return *this;
    }

    template<typename OtherSymmetricBlockMatrixType>
    This& operator=(const SymmetricBlockMatrixBlockExpr<OtherSymmetricBlockMatrixType>& other)
    {
      _doAssign(other);
      return *this;
    }
    
    This& operator=(const This& other)
    {
      // This version is required so GCC doesn't synthesize a default operator=.
      _doAssign(other);
      return *this;
    }

    template<typename OtherSymmetricBlockMatrixType>
    This& operator+=(const SymmetricBlockMatrixBlockExpr<OtherSymmetricBlockMatrixType>& other)
    {
      if(blockType_ == SelfAdjoint)
      {
        assert((BlockType)other.blockType() == SelfAdjoint);
        triangularView() += other.triangularView().nestedExpression();
      }
      else if(blockType_ == Plain)
      {
        assert((BlockType)other.blockType() == Plain || (BlockType)other.blockType() == Transposed);
        if((BlockType)other.blockType() == Transposed)
          myBlock_ += other.myBlock_.transpose();
        else
          myBlock_ += other.myBlock_;
      }
      else
      {
        assert((BlockType)other.blockType() == Plain || (BlockType)other.blockType() == Transposed);
        if((BlockType)other.blockType() == Transposed)
          myBlock_.transpose() += other.myBlock_.transpose();
        else
          myBlock_.transpose() += other.myBlock_;
      }
      return *this;
    }

  private:
    void initIndices(Index iBlock, Index jBlock, Index blockRows = 1, Index blockCols = 1)
    {
      if(iBlock == jBlock && blockRows == blockCols)
      {
        densei_ = xpr_.offset(iBlock);
        densej_ = densei_;
        if(blockRows > 0)
          xpr_.checkBlock(iBlock + blockRows - 1);
        denseRows_ = xpr_.offsetUnchecked(iBlock + blockRows) - densei_;
        if(blockCols > 0)
          xpr_.checkBlock(jBlock + blockCols - 1);
        denseCols_ = xpr_.offsetUnchecked(jBlock + blockCols) - densej_;
        blockType_ = SelfAdjoint;
      }
      else
      {
        if(jBlock > iBlock || (iBlock == jBlock && blockCols > blockRows))
        {
          densei_ = xpr_.offset(iBlock);
          densej_ = xpr_.offset(jBlock);
          if(blockRows > 0)
            xpr_.checkBlock(iBlock + blockRows - 1);
          denseRows_ = xpr_.offsetUnchecked(iBlock + blockRows) - densei_;
          if(blockCols > 0)
            xpr_.checkBlock(jBlock + blockCols - 1);
          denseCols_ = xpr_.offsetUnchecked(jBlock + blockCols) - densej_;
          blockType_ = Plain;
        }
        else
        {
          densei_ = xpr_.offset(jBlock);
          densej_ = xpr_.offset(iBlock);
          if(blockCols > 0)
            xpr_.checkBlock(jBlock + blockCols - 1);
          denseRows_ = xpr_.offsetUnchecked(jBlock + blockCols) - densei_;
          if(blockRows > 0)
            xpr_.checkBlock(iBlock + blockRows - 1);
          denseCols_ = xpr_.offsetUnchecked(iBlock + blockRows) - densej_;
          blockType_ = Transposed;
        }

        // Validate that the block does not cross below the diagonal (the indices have already been
        // flipped above the diagonal for ranges starting below the diagonal).
        if(densei_ + denseRows_ > densej_ + 1)
          throw std::invalid_argument("Off-diagonal block ranges may not cross the diagonal");
      }

      new (&myBlock_) Eigen::Block<DenseMatrixType>(xpr_.matrix_.block(densei_, densej_, denseRows_, denseCols_));
    }

    template<typename ScalarType>
    inline ScalarType coeffInternal(Index row, Index col) const
    {
      // We leave index checking up to the Block class
      if(blockType_ == Plain)
      {
        return myBlock_(row, col);
      }
      else if(blockType_ == SelfAdjoint)
      {
        if(row <= col)
          return myBlock_(row, col);
        else
          return myBlock_.transpose()(row, col);
      }
      else
      {
        return myBlock_.transpose()(row, col);
      }
    }
    
    template<typename OtherSymmetricBlockMatrixType>
    void _doAssign(const SymmetricBlockMatrixBlockExpr<OtherSymmetricBlockMatrixType>& other)
    {
      if(blockType_ == SelfAdjoint)
      {
        assert((BlockType)other.blockType() == SelfAdjoint);
        triangularView() = other.triangularView().nestedExpression();
      }
      else if(blockType_ == Plain)
      {
        assert((BlockType)other.blockType() == Plain || (BlockType)other.blockType() == Transposed);
        if((BlockType)other.blockType() == Transposed)
          myBlock_ = other.myBlock_.transpose();
        else
          myBlock_ = other.myBlock_;
      }
      else
      {
        assert((BlockType)other.blockType() == Plain || (BlockType)other.blockType() == Transposed);
        if((BlockType)other.blockType() == Transposed)
          myBlock_.transpose() = other.myBlock_.transpose();
        else
          myBlock_.transpose() = other.myBlock_;
      }
    }


  };

}

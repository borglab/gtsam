/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file    SymmetricBlockMatrix.h
* @brief   Access to matrices via blocks of pre-defined sizes.  Used in GaussianFactor and GaussianConditional.
* @author  Richard Roberts
* @date    Sep 18, 2010
*/
#pragma once

#include <gtsam/base/FastVector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/types.h>
#include <gtsam/dllexport.h>
#include <boost/serialization/nvp.hpp>
#include <cassert>
#include <stdexcept>
#include <array>

namespace boost {
namespace serialization {
class access;
} /* namespace serialization */
} /* namespace boost */

namespace gtsam {

  // Forward declarations
  class VerticalBlockMatrix;

  /**
  * This class stores a dense matrix and allows it to be accessed as a collection of blocks.  When
  * constructed, the caller must provide the dimensions of the blocks.
  *
  * The block structure is symmetric, but the underlying matrix does not necessarily need to be.
  *
  * This class also has a parameter that can be changed after construction to change the apparent
  * matrix view.  firstBlock() determines the block that appears to have index 0 for all operations
  * (except re-setting firstBlock()).
  *
  * @ingroup base */
  class GTSAM_EXPORT SymmetricBlockMatrix
  {
  public:
    typedef SymmetricBlockMatrix This;
    typedef Eigen::Block<Matrix> Block;
    typedef Eigen::Block<const Matrix> constBlock;

  protected:
    Matrix matrix_; ///< The full matrix
    FastVector<DenseIndex> variableColOffsets_; ///< the starting columns of each block (0-based)

    DenseIndex blockStart_; ///< Changes apparent matrix view, see main class comment.

  public:
    /// Construct from an empty matrix (asserts that the matrix is empty)
    SymmetricBlockMatrix() :
      blockStart_(0)
    {
      variableColOffsets_.push_back(0);
      assertInvariants();
    }

    /// Construct from a container of the sizes of each block.
    template<typename CONTAINER>
    SymmetricBlockMatrix(const CONTAINER& dimensions, bool appendOneDimension = false) :
      blockStart_(0)
    {
      fillOffsets(dimensions.begin(), dimensions.end(), appendOneDimension);
      matrix_.resize(variableColOffsets_.back(), variableColOffsets_.back());
      assertInvariants();
    }

    /// Construct from iterator over the sizes of each vertical block.
    template<typename ITERATOR>
    SymmetricBlockMatrix(ITERATOR firstBlockDim, ITERATOR lastBlockDim, bool appendOneDimension = false) :
      blockStart_(0)
    {
      fillOffsets(firstBlockDim, lastBlockDim, appendOneDimension);
      matrix_.resize(variableColOffsets_.back(), variableColOffsets_.back());
      assertInvariants();
    }

    /// Construct from a container of the sizes of each vertical block and a pre-prepared matrix.
    template<typename CONTAINER>
    SymmetricBlockMatrix(const CONTAINER& dimensions, const Matrix& matrix, bool appendOneDimension = false) :
      blockStart_(0)
    {
      matrix_.resize(matrix.rows(), matrix.cols());
      matrix_.triangularView<Eigen::Upper>() = matrix.triangularView<Eigen::Upper>();
      fillOffsets(dimensions.begin(), dimensions.end(), appendOneDimension);
      if(matrix_.rows() != matrix_.cols())
        throw std::invalid_argument("Requested to create a SymmetricBlockMatrix from a non-square matrix.");
      if(variableColOffsets_.back() != matrix_.cols())
        throw std::invalid_argument("Requested to create a SymmetricBlockMatrix with dimensions that do not sum to the total size of the provided matrix.");
      assertInvariants();
    }

    /// Copy the block structure, but do not copy the matrix data.  If blockStart() has been
    /// modified, this copies the structure of the corresponding matrix view. In the destination
    /// SymmetricBlockMatrix, blockStart() will be 0.
    static SymmetricBlockMatrix LikeActiveViewOf(const SymmetricBlockMatrix& other);

    /// Copy the block structure, but do not copy the matrix data. If blockStart() has been
    /// modified, this copies the structure of the corresponding matrix view. In the destination
    /// SymmetricBlockMatrix, blockStart() will be 0.
    static SymmetricBlockMatrix LikeActiveViewOf(const VerticalBlockMatrix& other);

    /// Row size
    DenseIndex rows() const { assertInvariants(); return variableColOffsets_.back() - variableColOffsets_[blockStart_]; }

    /// Column size
    DenseIndex cols() const { return rows(); }

    /// Block count
    DenseIndex nBlocks() const { return nActualBlocks() - blockStart_; }

    /// Number of dimensions for variable on this diagonal block.
    DenseIndex getDim(DenseIndex block) const {
      return calcIndices(block, block, 1, 1)[2];
    }

    /// @name Block getter methods.
    /// @{

    /// Get a copy of a block (anywhere in the matrix).
    /// This method makes a copy - use the methods below if performance is critical.
    Matrix block(DenseIndex I, DenseIndex J) const;

    /// Return the J'th diagonal block as a self adjoint view.
    Eigen::SelfAdjointView<Block, Eigen::Upper> diagonalBlock(DenseIndex J) {
      return block_(J, J).selfadjointView<Eigen::Upper>();
    }

    /// Return the J'th diagonal block as a self adjoint view.
    Eigen::SelfAdjointView<constBlock, Eigen::Upper> diagonalBlock(DenseIndex J) const {
      return block_(J, J).selfadjointView<Eigen::Upper>();
    }

    /// Get the diagonal of the J'th diagonal block.
    Vector diagonal(DenseIndex J) const {
      return block_(J, J).diagonal();
    }

    /// Get block above the diagonal (I, J).
    constBlock aboveDiagonalBlock(DenseIndex I, DenseIndex J) const {
      assert(I < J);
      return block_(I, J);
    }

    /// Return the square sub-matrix that contains blocks(i:j, i:j).
    Eigen::SelfAdjointView<constBlock, Eigen::Upper> selfadjointView(
        DenseIndex I, DenseIndex J) const {
      assert(J > I);
      return block_(I, I, J - I, J - I).selfadjointView<Eigen::Upper>();
    }

    /// Return the square sub-matrix that contains blocks(i:j, i:j) as a triangular view.
    Eigen::TriangularView<constBlock, Eigen::Upper> triangularView(DenseIndex I,
                                                                   DenseIndex J) const {
      assert(J > I);
      return block_(I, I, J - I, J - I).triangularView<Eigen::Upper>();
    }

    /// Get a range [i,j) from the matrix. Indices are in block units.
    constBlock aboveDiagonalRange(DenseIndex i_startBlock,
                                  DenseIndex i_endBlock,
                                  DenseIndex j_startBlock,
                                  DenseIndex j_endBlock) const {
      assert(i_startBlock < j_startBlock);
      assert(i_endBlock <= j_startBlock);
      return block_(i_startBlock, j_startBlock, i_endBlock - i_startBlock,
                   j_endBlock - j_startBlock);
    }

    /// Get a range [i,j) from the matrix. Indices are in block units.
    Block aboveDiagonalRange(DenseIndex i_startBlock, DenseIndex i_endBlock,
                             DenseIndex j_startBlock, DenseIndex j_endBlock) {
      assert(i_startBlock < j_startBlock);
      assert(i_endBlock <= j_startBlock);
      return block_(i_startBlock, j_startBlock, i_endBlock - i_startBlock,
                   j_endBlock - j_startBlock);
    }

    /// @}
    /// @name Block setter methods.
    /// @{

    /// Set a diagonal block. Only the upper triangular portion of `xpr` is evaluated.
    template <typename XprType>
    void setDiagonalBlock(DenseIndex I, const XprType& xpr) {
      block_(I, I).triangularView<Eigen::Upper>() = xpr.template triangularView<Eigen::Upper>();
    }

    /// Set an off-diagonal block. Only the upper triangular portion of `xpr` is evaluated.
    template <typename XprType>
    void setOffDiagonalBlock(DenseIndex I, DenseIndex J, const XprType& xpr) {
      assert(I != J);
      if (I < J) {
        block_(I, J) = xpr;
      } else {
        block_(J, I) = xpr.transpose();
      }
    }

    /// Increment the diagonal block by the values in `xpr`. Only reads the upper triangular part of `xpr`.
    template <typename XprType>
    void updateDiagonalBlock(DenseIndex I, const XprType& xpr) {
      // TODO(gareth): Eigen won't let us add triangular or self-adjoint views
      // here, so we do it manually.
      auto dest = block_(I, I);
      assert(dest.rows() == xpr.rows());
      assert(dest.cols() == xpr.cols());
      for (DenseIndex col = 0; col < dest.cols(); ++col) {
        for (DenseIndex row = 0; row <= col; ++row) {
          dest(row, col) += xpr(row, col);
        }
      }
    }

    /// Update an off diagonal block.
    /// NOTE(emmett): This assumes noalias().
    template <typename XprType>
    void updateOffDiagonalBlock(DenseIndex I, DenseIndex J, const XprType& xpr) {
      assert(I != J);
      if (I < J) {
        block_(I, J).noalias() += xpr;
      } else {
        block_(J, I).noalias() += xpr.transpose();
      }
    }

    /// @}
    /// @name Accessing the full matrix.
    /// @{

    /// Get self adjoint view.
    Eigen::SelfAdjointView<Block, Eigen::Upper> selfadjointView() {
      return full().selfadjointView<Eigen::Upper>();
    }

    /// Get self adjoint view.
    Eigen::SelfAdjointView<constBlock, Eigen::Upper> selfadjointView() const {
      return full().selfadjointView<Eigen::Upper>();
    }

    /// Set the entire active matrix. Only reads the upper triangular part of `xpr`.
    template <typename XprType>
    void setFullMatrix(const XprType& xpr) {
      full().triangularView<Eigen::Upper>() = xpr.template triangularView<Eigen::Upper>();
    }

    /// Set the entire active matrix zero.
    void setZero() {
      full().triangularView<Eigen::Upper>().setZero();
    }

    /// Negate the entire active matrix.
    void negate() {
      full().triangularView<Eigen::Upper>() *= -1.0;
    }

    /// Invert the entire active matrix in place.
    void invertInPlace() {
      const auto identity = Matrix::Identity(rows(), rows());
      full().triangularView<Eigen::Upper>() =
          selfadjointView()
              .llt()
              .solve(identity)
              .triangularView<Eigen::Upper>();
    }

    /// @}

    /// Retrieve or modify the first logical block, i.e. the block referenced by block index 0.
    /// Blocks before it will be inaccessible, except by accessing the underlying matrix using
    /// matrix().
    DenseIndex& blockStart() { return blockStart_; }

    /// Retrieve the first logical block, i.e. the block referenced by block index 0. Blocks before
    /// it will be inaccessible, except by accessing the underlying matrix using matrix().
    DenseIndex blockStart() const { return blockStart_; }

    /**
     * Given the augmented Hessian [A1'A1 A1'A2 A1'b
     *                              A2'A1 A2'A2 A2'b
     *                               b'A1  b'A2  b'b]
     * on x1 and x2, does partial Cholesky in-place to obtain [R Sd;0 L]  such that
     *   R'R  = A1'A1
     *   R'Sd = [A1'A2 A1'b]
     *   L'L is the augmented Hessian on the the separator x2
     * R and Sd can be interpreted as a GaussianConditional |R*x1 + S*x2 - d]^2
     */
    void choleskyPartial(DenseIndex nFrontals);

    /**
     * After partial Cholesky, we can optionally split off R and Sd, to be interpreted as
     * a GaussianConditional |R*x1 + S*x2 - d]^2. We leave the symmetric lower block L in place,
     * and adjust block_start so now *this refers to it.
     */
    VerticalBlockMatrix split(DenseIndex nFrontals);

    /// Number of offsets in the full matrix.
    DenseIndex nOffsets() const {
      return variableColOffsets_.size();
    }

    /// Number of actual blocks in the full matrix.
    DenseIndex nActualBlocks() const {
      return nOffsets() - 1;
    }

    /// Get an offset for a block index (in the active view).
    DenseIndex offset(DenseIndex block) const {
      assert(block >= 0);
      const DenseIndex actual_index = block + blockStart();
      assert(actual_index < nOffsets());
      return variableColOffsets_[actual_index];
    }

    protected:

    /// Get an arbitrary block from the matrix. Indices are in block units.
    constBlock block_(DenseIndex iBlock, DenseIndex jBlock,
                      DenseIndex blockRows = 1, DenseIndex blockCols = 1) const {
      const std::array<DenseIndex, 4> indices =
          calcIndices(iBlock, jBlock, blockRows, blockCols);
      return matrix_.block(indices[0], indices[1], indices[2], indices[3]);
    }

    /// Get an arbitrary block from the matrix. Indices are in block units.
    Block block_(DenseIndex iBlock, DenseIndex jBlock, DenseIndex blockRows = 1,
                 DenseIndex blockCols = 1) {
      const std::array<DenseIndex, 4> indices =
          calcIndices(iBlock, jBlock, blockRows, blockCols);
      return matrix_.block(indices[0], indices[1], indices[2], indices[3]);
    }

    /// Get the full matrix as a block.
    constBlock full() const {
      return block_(0, 0, nBlocks(), nBlocks());
    }

    /// Get the full matrix as a block.
    Block full() {
      return block_(0, 0, nBlocks(), nBlocks());
    }

    /// Compute the indices into the underlying matrix for a given block.
    std::array<DenseIndex, 4> calcIndices(DenseIndex iBlock, DenseIndex jBlock,
                                          DenseIndex blockRows,
                                          DenseIndex blockCols) const {
      assert(blockRows >= 0);
      assert(blockCols >= 0);

      // adjust indices to account for start and size of blocks
      const DenseIndex denseI = offset(iBlock);
      const DenseIndex denseJ = offset(jBlock);
      const DenseIndex denseRows = offset(iBlock + blockRows) - denseI;
      const DenseIndex denseCols = offset(jBlock + blockCols) - denseJ;
      return {{denseI, denseJ, denseRows, denseCols}};
    }

    void assertInvariants() const
    {
      assert(matrix_.rows() == matrix_.cols());
      assert(matrix_.cols() == variableColOffsets_.back());
      assert(blockStart_ < (DenseIndex)variableColOffsets_.size());
    }

    template<typename ITERATOR>
    void fillOffsets(ITERATOR firstBlockDim, ITERATOR lastBlockDim, bool appendOneDimension)
    {
      variableColOffsets_.resize((lastBlockDim-firstBlockDim) + 1 + (appendOneDimension ? 1 : 0));
      variableColOffsets_[0] = 0;
      DenseIndex j=0;
      for(ITERATOR dim=firstBlockDim; dim!=lastBlockDim; ++dim) {
        variableColOffsets_[j+1] = variableColOffsets_[j] + *dim;
        ++ j;
      }
      if(appendOneDimension)
      {
        variableColOffsets_[j+1] = variableColOffsets_[j] + 1;
        ++ j;
      }
    }

    friend class VerticalBlockMatrix;
    template<typename SymmetricBlockMatrixType> friend class SymmetricBlockMatrixBlockExpr;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      // Fill in the lower triangle part of the matrix, so boost::serialization won't
      // complain about uninitialized data with an input_stream_error exception
      // http://www.boost.org/doc/libs/1_37_0/libs/serialization/doc/exceptions.html#stream_error
      matrix_.triangularView<Eigen::Lower>() = matrix_.triangularView<Eigen::Upper>().transpose();
      ar & BOOST_SERIALIZATION_NVP(matrix_);
      ar & BOOST_SERIALIZATION_NVP(variableColOffsets_);
      ar & BOOST_SERIALIZATION_NVP(blockStart_);
    }
  };

  /// Foward declare exception class
  class CholeskyFailed;

}

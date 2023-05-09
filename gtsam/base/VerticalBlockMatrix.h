/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VerticalBlockMatrix.h
 * @brief   A matrix with column blocks of pre-defined sizes.  Used in JacobianFactor and
 *          GaussianConditional.
 * @author  Richard Roberts
 * @date    Sep 18, 2010 */
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastVector.h>

namespace gtsam {

  // Forward declarations
  class SymmetricBlockMatrix;

  /**
   * This class stores a dense matrix and allows it to be accessed as a collection of vertical
   * blocks. The dimensions of the blocks are provided when constructing this class.
   *
   * This class also has three parameters that can be changed after construction that change the
   * apparent view of the matrix without any reallocation or data copying.  firstBlock() determines
   * the block that has index 0 for all operations (except for re-setting firstBlock()).  rowStart()
   * determines the apparent first row of the matrix for all operations (except for setting
   * rowStart() and rowEnd()).  rowEnd() determines the apparent exclusive (one-past-the-last) last
   * row for all operations.  To include all rows, rowEnd() should be set to the number of rows in
   * the matrix (i.e. one after the last true row index).
   *
   * @addtogroup base */
  class GTSAM_EXPORT VerticalBlockMatrix
  {
  public:
    typedef VerticalBlockMatrix This;
    typedef Eigen::Block<Matrix> Block;
    typedef Eigen::Block<const Matrix> constBlock;

  protected:
    Matrix matrix_; ///< The full matrix
    FastVector<DenseIndex> variableColOffsets_; ///< the starting columns of each block (0-based)

    DenseIndex rowStart_; ///< Changes apparent matrix view, see main class comment.
    DenseIndex rowEnd_; ///< Changes apparent matrix view, see main class comment.
    DenseIndex blockStart_; ///< Changes apparent matrix view, see main class comment.

  public:

    /** Construct an empty VerticalBlockMatrix */
    VerticalBlockMatrix() :
      rowStart_(0), rowEnd_(0), blockStart_(0)
    {
      variableColOffsets_.push_back(0);
      assertInvariants();
    }

    /** Construct from a container of the sizes of each vertical block. */
    template<typename CONTAINER>
    VerticalBlockMatrix(const CONTAINER& dimensions, DenseIndex height,
        bool appendOneDimension = false) :
        variableColOffsets_(dimensions.size() + (appendOneDimension ? 2 : 1)),
        rowStart_(0), rowEnd_(height), blockStart_(0) {
      fillOffsets(dimensions.begin(), dimensions.end(), appendOneDimension);
      matrix_.resize(height, variableColOffsets_.back());
      assertInvariants();
    }

    /** Construct from a container of the sizes of each vertical block and a pre-prepared matrix. */
    template<typename CONTAINER, typename DERIVED>
    VerticalBlockMatrix(const CONTAINER& dimensions,
        const Eigen::MatrixBase<DERIVED>& matrix, bool appendOneDimension = false) :
        matrix_(matrix), variableColOffsets_(dimensions.size() + (appendOneDimension ? 2 : 1)),
        rowStart_(0), rowEnd_(matrix.rows()), blockStart_(0) {
      fillOffsets(dimensions.begin(), dimensions.end(), appendOneDimension);
      if (variableColOffsets_.back() != matrix_.cols())
        throw std::invalid_argument(
            "Requested to create a VerticalBlockMatrix with dimensions that do not sum to the total columns of the provided matrix.");
      assertInvariants();
    }

    /** Construct from iterator over the sizes of each vertical block. */
    template<typename ITERATOR>
    VerticalBlockMatrix(ITERATOR firstBlockDim, ITERATOR lastBlockDim,
        DenseIndex height, bool appendOneDimension = false) :
        variableColOffsets_((lastBlockDim-firstBlockDim) + (appendOneDimension ? 2 : 1)),
        rowStart_(0), rowEnd_(height), blockStart_(0) {
      fillOffsets(firstBlockDim, lastBlockDim, appendOneDimension);
      matrix_.resize(height, variableColOffsets_.back());
      assertInvariants();
    }

    /** Copy the block structure and resize the underlying matrix, but do not copy the matrix data.
    *  If blockStart(), rowStart(), and/or rowEnd() have been modified, this copies the structure of
    *  the corresponding matrix view. In the destination VerticalBlockView, blockStart() and
    *  rowStart() will thus be 0, rowEnd() will be cols() of the source VerticalBlockView, and the
    *  underlying matrix will be the size of the view of the source matrix.  */
    static VerticalBlockMatrix LikeActiveViewOf(const VerticalBlockMatrix& rhs);

    /** Copy the block structure, but do not copy the matrix data. If blockStart() has been
    *   modified, this copies the structure of the corresponding matrix view. In the destination
    *   VerticalBlockMatrix, blockStart() will be 0. */
    static VerticalBlockMatrix LikeActiveViewOf(const SymmetricBlockMatrix& rhs, DenseIndex height);

    /// Row size
    DenseIndex rows() const { assertInvariants(); return rowEnd_ - rowStart_; }

    /// Column size
    DenseIndex cols() const { assertInvariants(); return variableColOffsets_.back() - variableColOffsets_[blockStart_]; }

    /// Block count
    DenseIndex nBlocks() const { assertInvariants(); return variableColOffsets_.size() - 1 - blockStart_; }

    /** Access a single block in the underlying matrix with read/write access */
    Block operator()(DenseIndex block) { return range(block, block+1); }

    /** Access a const block view */
    const constBlock operator()(DenseIndex block) const { return range(block, block+1); }

    /** access ranges of blocks at a time */
    Block range(DenseIndex startBlock, DenseIndex endBlock) {
      assertInvariants();
      DenseIndex actualStartBlock = startBlock + blockStart_;
      DenseIndex actualEndBlock = endBlock + blockStart_;
      if(startBlock != 0 || endBlock != 0) {
        checkBlock(actualStartBlock);
        assert(actualEndBlock < (DenseIndex)variableColOffsets_.size());
      }
      const DenseIndex startCol = variableColOffsets_[actualStartBlock];
      const DenseIndex rangeCols = variableColOffsets_[actualEndBlock] - startCol;
      return matrix_.block(rowStart_, startCol, this->rows(), rangeCols);
    }

    const constBlock range(DenseIndex startBlock, DenseIndex endBlock) const {
      assertInvariants();
      DenseIndex actualStartBlock = startBlock + blockStart_;
      DenseIndex actualEndBlock = endBlock + blockStart_;
      if(startBlock != 0 || endBlock != 0) {
        checkBlock(actualStartBlock);
        assert(actualEndBlock < (DenseIndex)variableColOffsets_.size());
      }
      const DenseIndex startCol = variableColOffsets_[actualStartBlock];
      const DenseIndex rangeCols = variableColOffsets_[actualEndBlock] - startCol;
      return ((const Matrix&)matrix_).block(rowStart_, startCol, this->rows(), rangeCols);
    }

    /** Return the full matrix, *not* including any portions excluded by rowStart(), rowEnd(), and firstBlock() */
    Block full() { return range(0, nBlocks()); }

    /** Return the full matrix, *not* including any portions excluded by rowStart(), rowEnd(), and firstBlock() */
    const constBlock full() const { return range(0, nBlocks()); }

    DenseIndex offset(DenseIndex block) const {
      assertInvariants();
      DenseIndex actualBlock = block + blockStart_;
      checkBlock(actualBlock);
      return variableColOffsets_[actualBlock];
    }

    /** Get the apparent first row of the underlying matrix for all operations */
    const DenseIndex& rowStart() const { return rowStart_; }

    /** Get or set the apparent first row of the underlying matrix for all operations */
    DenseIndex& rowStart() { return rowStart_; }

    /** Get the apparent last row (exclusive, i.e. rows() == rowEnd() - rowStart()) of the underlying matrix for all operations */
    const DenseIndex& rowEnd() const { return rowEnd_; }

    /** Get or set the apparent last row (exclusive, i.e. rows() == rowEnd() - rowStart()) of the underlying matrix for all operations */
    DenseIndex& rowEnd() { return rowEnd_; }

    /** Get the apparent first block for all operations */
    const DenseIndex& firstBlock() const { return blockStart_; }

    /** Get or set the apparent first block for all operations */
    DenseIndex& firstBlock() { return blockStart_; }

    /** Access to full matrix (*including* any portions excluded by rowStart(), rowEnd(), and firstBlock()) */
    const Matrix& matrix() const { return matrix_; }

    /** Non-const access to full matrix (*including* any portions excluded by rowStart(), rowEnd(), and firstBlock()) */
    Matrix& matrix() { return matrix_; }

  protected:
    void assertInvariants() const {
      assert(matrix_.cols() == variableColOffsets_.back());
      assert(blockStart_ < (DenseIndex)variableColOffsets_.size());
      assert(rowStart_ <= matrix_.rows());
      assert(rowEnd_ <= matrix_.rows());
      assert(rowStart_ <= rowEnd_);
    }

    void checkBlock(DenseIndex block) const {
      static_cast<void>(block); //Disable unused varibale warnings.
      assert(matrix_.cols() == variableColOffsets_.back());
      assert(block < (DenseIndex)variableColOffsets_.size() - 1);
      assert(variableColOffsets_[block] < matrix_.cols() && variableColOffsets_[block+1] <= matrix_.cols());
    }

    template<typename ITERATOR>
    void fillOffsets(ITERATOR firstBlockDim, ITERATOR lastBlockDim, bool appendOneDimension) {
      variableColOffsets_[0] = 0;
      DenseIndex j=0;
      for(ITERATOR dim=firstBlockDim; dim!=lastBlockDim; ++dim, ++j)
        variableColOffsets_[j+1] = variableColOffsets_[j] + *dim;
      if(appendOneDimension)
        variableColOffsets_[j+1] = variableColOffsets_[j] + 1;
    }

    friend class SymmetricBlockMatrix;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(matrix_);
      ar & BOOST_SERIALIZATION_NVP(variableColOffsets_);
      ar & BOOST_SERIALIZATION_NVP(rowStart_);
      ar & BOOST_SERIALIZATION_NVP(rowEnd_);
      ar & BOOST_SERIALIZATION_NVP(blockStart_);
    }
  };

}

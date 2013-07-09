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
 *        GaussianConditional.
 * @author  Richard Roberts
 * @date    Sep 18, 2010 */
#pragma once

#include <gtsam/base/Matrix.h>

namespace gtsam {

  // Forward declarations
  //class S

  /**
   * This class stores a dense matrix and allows it to be accessed as a collection of vertical blocks.
   * It also provides for accessing individual columns from those blocks.  When constructed or
   * resized, the caller must provide the dimensions of the blocks.
   *
   * This class also has three parameters that can be changed after construction that change the
   * apparent view of the matrix without any reallocation or data copying.  firstBlock() determines
   * the block that has index 0 for all operations (except for re-setting firstBlock()).  rowStart()
   * determines the apparent first row of the matrix for all operations (except for setting rowStart()
   * and rowEnd()).  rowEnd() determines the apparent exclusive (one-past-the-last) last row for all
   * operations.  To include all rows, rowEnd() should be set to the number of rows in the matrix
   * (i.e. one after the last true row index).
   *
   * @addtogroup base */
  class VerticalBlockMatrix {
  public:
    typedef VerticalBlockMatrix This;
    typedef Matrix::Index Index;
    typedef Eigen::Block<Matrix> Block;
    typedef Eigen::Block<const Matrix> constBlock;

    // columns of blocks
    typedef Eigen::Block<Matrix>::ColXpr Column;
    typedef Eigen::Block<const Matrix>::ConstColXpr constColumn;

  protected:
    Matrix matrix_; ///< The full matrix
    std::vector<Index> variableColOffsets_; ///< the starting columns of each block (0-based)

    Index rowStart_; ///< Changes apparent matrix view, see main class comment.
    Index rowEnd_; ///< Changes apparent matrix view, see main class comment.
    Index blockStart_; ///< Changes apparent matrix view, see main class comment.

  public:

    /** Construct an empty VerticalBlockMatrix */
    VerticalBlockMatrix() :
      rowStart_(0), rowEnd_(0), blockStart_(0)
    {
      variableColOffsets_.push_back(0);
    }

    /**
     * Construct from a container of the sizes of each vertical block, resize the matrix so that its
     * height is matrixNewHeight and its width fits the given block dimensions. */
    template<typename CONTAINER>
    VerticalBlockMatrix(const CONTAINER dimensions, DenseIndex height) :
      rowStart_(0), rowEnd_(height), blockStart_(0)
    {
      fillOffsets(dimensions.begin(), dimensions.end());
      matrix_.resize(height, variableColOffsets_.back());
      assertInvariants();
    }

    /**
     * Construct from iterator over the sizes of each vertical block, resize the matrix so that its
     * height is matrixNewHeight and its width fits the given block dimensions. */
    template<typename ITERATOR>
    VerticalBlockMatrix(ITERATOR firstBlockDim, ITERATOR lastBlockDim, DenseIndex height) :
      rowStart_(0), rowEnd_(height), blockStart_(0)
    {
      fillOffsets(firstBlockDim, lastBlockDim);
      matrix_.resize(height, variableColOffsets_.back());
      assertInvariants();
    }

    /// Row size
    Index rows() const { assertInvariants(); return rowEnd_ - rowStart_; }

    /// Column size
    Index cols() const { assertInvariants(); return variableColOffsets_.back() - variableColOffsets_[blockStart_]; }

    /// Block count
    Index nBlocks() const { assertInvariants(); return variableColOffsets_.size() - 1 - blockStart_; }

    /** Access a single block in the underlying matrix with read/write access */
    Block operator()(Index block) { return range(block, block+1); }

    /** Access a const block view */
    const constBlock operator()(Index block) const { return range(block, block+1); }

    /** access ranges of blocks at a time */
    Block range(Index startBlock, Index endBlock) {
      assertInvariants();
      Index actualStartBlock = startBlock + blockStart_;
      Index actualEndBlock = endBlock + blockStart_;
      if(startBlock != 0 || endBlock != 0) {
        checkBlock(actualStartBlock);
        assert(actualEndBlock < (DenseIndex)variableColOffsets_.size());
      }
      const Index startCol = variableColOffsets_[actualStartBlock];
      const Index rangeCols = variableColOffsets_[actualEndBlock] - startCol;
      return matrix_.block(rowStart_, startCol, this->rows(), rangeCols);
    }

    const constBlock range(Index startBlock, Index endBlock) const {
      assertInvariants();
      Index actualStartBlock = startBlock + blockStart_;
      Index actualEndBlock = endBlock + blockStart_;
      if(startBlock != 0 || endBlock != 0) {
        checkBlock(actualStartBlock);
        assert(actualEndBlock < (DenseIndex)variableColOffsets_.size());
      }
      const Index startCol = variableColOffsets_[actualStartBlock];
      const Index rangeCols = variableColOffsets_[actualEndBlock] - startCol;
      return ((const Matrix&)matrix_).block(rowStart_, startCol, this->rows(), rangeCols);
    }

    /** Return the full matrix, *not* including any portions excluded by rowStart(), rowEnd(), and firstBlock() */
    Block full() { return range(0, nBlocks()); }

    /** Return the full matrix, *not* including any portions excluded by rowStart(), rowEnd(), and firstBlock() */
    const constBlock full() const { return range(0, nBlocks()); }

    Index offset(Index block) const {
      assertInvariants();
      Index actualBlock = block + blockStart_;
      checkBlock(actualBlock);
      return variableColOffsets_[actualBlock];
    }

    /** Get or set the apparent first row of the underlying matrix for all operations */
    Index& rowStart() { return rowStart_; }

    /** Get or set the apparent last row (exclusive, i.e. rows() == rowEnd() - rowStart()) of the underlying matrix for all operations */
    Index& rowEnd() { return rowEnd_; }

    /** Get or set the apparent first block for all operations */
    Index& firstBlock() { return blockStart_; }

    /** Get the apparent first row of the underlying matrix for all operations */
    Index rowStart() const { return rowStart_; }

    /** Get the apparent last row (exclusive, i.e. rows() == rowEnd() - rowStart()) of the underlying matrix for all operations */
    Index rowEnd() const { return rowEnd_; }

    /** Get the apparent first block for all operations */
    Index firstBlock() const { return blockStart_; }

    /** Access to full matrix (*including* any portions excluded by rowStart(), rowEnd(), and firstBlock()) */
    const Matrix& matrix() const { return matrix_; }

    /** Non-const access to full matrix (*including* any portions excluded by rowStart(), rowEnd(), and firstBlock()) */
    Matrix& matrix() { return matrix_; }

    /**
    * Copy the block structure and resize the underlying matrix, but do not
    * copy the matrix data.  If blockStart(), rowStart(), and/or rowEnd() have
    * been modified, this copies the structure of the corresponding matrix view.
    * In the destination VerticalBlockView, blockStart() and rowStart() will
    * thus be 0, rowEnd() will be cols() of the source VerticalBlockView, and
    * the underlying matrix will be the size of the view of the source matrix.
    */
    static VerticalBlockMatrix LikeActiveViewOf(const VerticalBlockMatrix& rhs);

  protected:
    void assertInvariants() const {
      assert(matrix_.cols() == variableColOffsets_.back());
      assert(blockStart_ < (DenseIndex)variableColOffsets_.size());
      assert(rowStart_ <= matrix_.rows());
      assert(rowEnd_ <= matrix_.rows());
      assert(rowStart_ <= rowEnd_);
    }

    void checkBlock(DenseIndex block) const {
      assert(matrix_.cols() == variableColOffsets_.back());
      assert(block < (DenseIndex)variableColOffsets_.size() - 1);
      assert(variableColOffsets_[block] < matrix_.cols() && variableColOffsets_[block+1] <= matrix_.cols());
    }

    template<typename ITERATOR>
    void fillOffsets(ITERATOR firstBlockDim, ITERATOR lastBlockDim) {
      variableColOffsets_.resize((lastBlockDim-firstBlockDim)+1);
      variableColOffsets_[0] = 0;
      Index j=0;
      for(ITERATOR dim=firstBlockDim; dim!=lastBlockDim; ++dim) {
        variableColOffsets_[j+1] = variableColOffsets_[j] + *dim;
        ++ j;
      }
    }

    //friend class SymmetricBlockView;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(matrix_);
      ar & BOOST_SERIALIZATION_NVP(variableColOffsets_);
      ar & BOOST_SERIALIZATION_NVP(rowStart_);
      ar & BOOST_SERIALIZATION_NVP(rowEnd_);
      ar & BOOST_SERIALIZATION_NVP(blockStart_);
    }
  };

}

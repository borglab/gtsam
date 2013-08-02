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

#include <gtsam/base/Matrix.h>

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
  * @addtogroup base */
  class SymmetricBlockMatrix
  {
  public:
    typedef SymmetricBlockMatrix This;
    typedef Eigen::Block<Matrix> Block;
    typedef Eigen::Block<const Matrix> constBlock;

  protected:
    Matrix matrix_; ///< The full matrix
    std::vector<DenseIndex> variableColOffsets_; ///< the starting columns of each block (0-based)

    DenseIndex blockStart_; ///< Changes apparent matrix view, see main class comment.

  public:
    /** Construct from an empty matrix (asserts that the matrix is empty) */
    SymmetricBlockMatrix() :
      blockStart_(0)
    {
      variableColOffsets_.push_back(0);
      assertInvariants();
    }

    /** Construct from a container of the sizes of each block. */
    template<typename CONTAINER>
    SymmetricBlockMatrix(const CONTAINER dimensions) :
      blockStart_(0)
    {
      fillOffsets(dimensions.begin(), dimensions.end());
      matrix_.resize(variableColOffsets_.back(), variableColOffsets_.back());
      assertInvariants();
    }

    /**
    * Construct from iterator over the sizes of each vertical block. */
    template<typename ITERATOR>
    SymmetricBlockMatrix(ITERATOR firstBlockDim, ITERATOR lastBlockDim) :
      blockStart_(0)
    {
      fillOffsets(firstBlockDim, lastBlockDim);
      matrix_.resize(variableColOffsets_.back(), variableColOffsets_.back());
      assertInvariants();
    }

    /** Construct from a container of the sizes of each vertical block and a pre-prepared matrix. */
    template<typename CONTAINER>
    SymmetricBlockMatrix(const CONTAINER dimensions, const Matrix& matrix) :
      matrix_(matrix), blockStart_(0)
    {
      fillOffsets(dimensions.begin(), dimensions.end());
      if(matrix_.rows() != matrix_.cols())
        throw std::invalid_argument("Requested to create a SymmetricBlockMatrix from a non-square matrix.");
      if(variableColOffsets_.back() != matrix_.cols())
        throw std::invalid_argument("Requested to create a SymmetricBlockMatrix with dimensions that do not sum to the total size of the provided matrix.");
      assertInvariants();
    }
    
    /** Copy the block structure, but do not copy the matrix data.  If blockStart() has been
    *   modified, this copies the structure of the corresponding matrix view. In the destination
    *   SymmetricBlockMatrix, blockStart() will be 0. */
    static SymmetricBlockMatrix LikeActiveViewOf(const SymmetricBlockMatrix& other);
    
    /** Copy the block structure, but do not copy the matrix data. If blockStart() has been
    *   modified, this copies the structure of the corresponding matrix view. In the destination
    *   SymmetricBlockMatrix, blockStart() will be 0. */
    static SymmetricBlockMatrix LikeActiveViewOf(const VerticalBlockMatrix& other);

    /** Row size */
    DenseIndex rows() const { assertInvariants(); return variableColOffsets_.back() - variableColOffsets_[blockStart_]; }

    /** Column size */
    DenseIndex cols() const { return rows(); }


    /** Block count */
    DenseIndex nBlocks() const { assertInvariants(); return variableColOffsets_.size() - 1 - blockStart_; }

    Block operator()(DenseIndex i_block, DenseIndex j_block) {
      return range(i_block, i_block+1, j_block, j_block+1);
    }

    constBlock operator()(DenseIndex i_block, DenseIndex j_block) const {
      return range(i_block, i_block+1, j_block, j_block+1);
    }

    Block range(DenseIndex i_startBlock, DenseIndex i_endBlock, DenseIndex j_startBlock, DenseIndex j_endBlock) {
      assertInvariants();
      DenseIndex i_actualStartBlock = i_startBlock + blockStart_;
      DenseIndex i_actualEndBlock = i_endBlock + blockStart_;
      DenseIndex j_actualStartBlock = j_startBlock + blockStart_;
      DenseIndex j_actualEndBlock = j_endBlock + blockStart_;
      checkBlock(i_actualStartBlock);
      checkBlock(j_actualStartBlock);
      if(i_startBlock != 0 || i_endBlock != 0) {
        checkBlock(i_actualStartBlock);
        assert(i_actualEndBlock < (DenseIndex)variableColOffsets_.size());
      }
      if(j_startBlock != 0 || j_endBlock != 0) {
        checkBlock(j_actualStartBlock);
        assert(j_actualEndBlock < (DenseIndex)variableColOffsets_.size());
      }
      return matrix_.block(
        variableColOffsets_[i_actualStartBlock], variableColOffsets_[j_actualStartBlock],
        variableColOffsets_[i_actualEndBlock] - variableColOffsets_[i_actualStartBlock],
        variableColOffsets_[j_actualEndBlock] - variableColOffsets_[j_actualStartBlock]);
    }

    constBlock range(DenseIndex i_startBlock, DenseIndex i_endBlock, DenseIndex j_startBlock, DenseIndex j_endBlock) const {
      assertInvariants();
      DenseIndex i_actualStartBlock = i_startBlock + blockStart_;
      DenseIndex i_actualEndBlock = i_endBlock + blockStart_;
      DenseIndex j_actualStartBlock = j_startBlock + blockStart_;
      DenseIndex j_actualEndBlock = j_endBlock + blockStart_;
      checkBlock(i_actualStartBlock);
      checkBlock(j_actualStartBlock);
      if(i_startBlock != 0 || i_endBlock != 0) {
        checkBlock(i_actualStartBlock);
        assert(i_actualEndBlock < (DenseIndex)variableColOffsets_.size());
      }
      if(j_startBlock != 0 || j_endBlock != 0) {
        checkBlock(j_actualStartBlock);
        assert(j_actualEndBlock < (DenseIndex)variableColOffsets_.size());
      }
      return matrix_.block(
        variableColOffsets_[i_actualStartBlock], variableColOffsets_[j_actualStartBlock],
        variableColOffsets_[i_actualEndBlock] - variableColOffsets_[i_actualStartBlock],
        variableColOffsets_[j_actualEndBlock] - variableColOffsets_[j_actualStartBlock]);
    }

    /** Return the full matrix, *not* including any portions excluded by firstBlock(). */
    Block full() {
      return range(0,nBlocks(), 0,nBlocks());
    }

    /** Return the full matrix, *not* including any portions excluded by firstBlock(). */
    constBlock full() const {
      return range(0,nBlocks(), 0,nBlocks());
    }

    /** Access to full matrix, including any portions excluded by firstBlock() to other operations. */
    const Matrix& matrix() const { return matrix_; }

    /** Access to full matrix, including any portions excluded by firstBlock() to other operations. */
    Matrix& matrix() { return matrix_; }

    DenseIndex offset(DenseIndex block) const {
      assertInvariants();
      DenseIndex actualBlock = block + blockStart_;
      checkBlock(actualBlock);
      return variableColOffsets_[actualBlock];
    }

    DenseIndex& blockStart() { return blockStart_; }
    DenseIndex blockStart() const { return blockStart_; }

  protected:
    void assertInvariants() const {
      assert(matrix_.rows() == matrix_.cols());
      assert(matrix_.cols() == variableColOffsets_.back());
      assert(blockStart_ < (DenseIndex)variableColOffsets_.size());
    }

    void checkBlock(DenseIndex block) const {
      assert(matrix_.rows() == matrix_.cols());
      assert(matrix_.cols() == variableColOffsets_.back());
      assert(block < (DenseIndex)variableColOffsets_.size()-1);
      assert(variableColOffsets_[block] < matrix_.cols() && variableColOffsets_[block+1] <= matrix_.cols());
    }

    template<typename ITERATOR>
    void fillOffsets(ITERATOR firstBlockDim, ITERATOR lastBlockDim) {
      variableColOffsets_.resize((lastBlockDim-firstBlockDim)+1);
      variableColOffsets_[0] = 0;
      DenseIndex j=0;
      for(ITERATOR dim=firstBlockDim; dim!=lastBlockDim; ++dim) {
        variableColOffsets_[j+1] = variableColOffsets_[j] + *dim;
        ++ j;
      }
    }

    friend class VerticalBlockMatrix;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(matrix_);
      ar & BOOST_SERIALIZATION_NVP(variableColOffsets_);
      ar & BOOST_SERIALIZATION_NVP(blockStart_);
    }
  };


}

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    blockMatrices.h
 * @brief   Access to matrices via blocks of pre-defined sizes.  Used in GaussianFactor and GaussianConditional.
 * @author  Richard Roberts
 * @date    Sep 18, 2010
 */
#pragma once

#include <gtsam/base/Matrix.h>

namespace gtsam {

template<class MATRIX> class SymmetricBlockView;

/**
 * This class stores a *reference* to a matrix and allows it to be accessed as
 * a collection of vertical blocks.  It also provides for accessing individual
 * columns from those blocks.  When constructed or resized, the caller must
 * provide the dimensions of the blocks, as well as an underlying matrix
 * storage object.  This class will resize the underlying matrix such that it
 * is consistent with the given block dimensions.
 *
 * This class also has three parameters that can be changed after construction
 * that change the apparent view of the matrix.  firstBlock() determines the
 * block that has index 0 for all operations (except for re-setting
 * firstBlock()).  rowStart() determines the apparent first row of the matrix
 * for all operations (except for setting rowStart() and rowEnd()).  rowEnd()
 * determines the apparent *exclusive* last row for all operations.  To include
 * all rows, rowEnd() should be set to the number of rows in the matrix (i.e.
 * one after the last true row index).
 *
 * @addtogroup base
 */
template<class MATRIX>
class VerticalBlockView {
public:
  typedef MATRIX FullMatrix;
  typedef Eigen::Block<MATRIX> Block;
  typedef Eigen::Block<const MATRIX> constBlock;

  // columns of blocks
  typedef Eigen::VectorBlock<typename MATRIX::ColXpr> Column;
  typedef Eigen::VectorBlock<const typename MATRIX::ConstColXpr> constColumn;

protected:
  FullMatrix& matrix_; // the reference to the full matrix
  std::vector<size_t> variableColOffsets_; // the starting columns of each block (0-based)

  // Changes apparent matrix view, see main class comment.
  size_t rowStart_; // 0 initially
  size_t rowEnd_; // the number of row - 1, initially
  size_t blockStart_; // 0 initially

public:
  /** Construct from an empty matrix (asserts that the matrix is empty) */
  VerticalBlockView(FullMatrix& matrix) :
    matrix_(matrix), rowStart_(0), rowEnd_(matrix_.rows()), blockStart_(0) {
    fillOffsets((size_t*)0, (size_t*)0);
    assertInvariants();
  }

  /**
   * Construct from a non-empty matrix and copy the block structure from
   * another block view.
   */
  template<class RHS>
  VerticalBlockView(FullMatrix& matrix, const RHS& rhs) :
    matrix_(matrix) {
    if((size_t) matrix_.rows() != rhs.rows() || (size_t) matrix_.cols() != rhs.cols())
      throw std::invalid_argument(
          "In VerticalBlockView<>(FullMatrix& matrix, const RHS& rhs), matrix and rhs must\n"
          "already be of the same size.  If not, construct the VerticalBlockView from an\n"
          "empty matrix and then use copyStructureFrom(const RHS& rhs) to resize the matrix\n"
          "and set up the block structure.");
    copyStructureFrom(rhs);
    assertInvariants();
  }

  /** Construct from iterators over the sizes of each vertical block */
  template<typename ITERATOR>
  VerticalBlockView(FullMatrix& matrix, ITERATOR firstBlockDim, ITERATOR lastBlockDim) :
  matrix_(matrix), rowStart_(0), rowEnd_(matrix_.rows()), blockStart_(0) {
    fillOffsets(firstBlockDim, lastBlockDim);
    assertInvariants();
  }

  /**
   * Construct from a vector of the sizes of each vertical block, resize the
   * matrix so that its height is matrixNewHeight and its width fits the given
   * block dimensions.
   */
  template<typename ITERATOR>
  VerticalBlockView(FullMatrix& matrix, ITERATOR firstBlockDim, ITERATOR lastBlockDim, size_t matrixNewHeight) :
  matrix_(matrix), rowStart_(0), rowEnd_(matrixNewHeight), blockStart_(0) {
    fillOffsets(firstBlockDim, lastBlockDim);
    matrix_.resize(matrixNewHeight, variableColOffsets_.back());
    assertInvariants();
  }

  /** Row size 
   */
  size_t rows() const { assertInvariants(); return rowEnd_ - rowStart_; }
  
  /** Column size
   */ 
  size_t cols() const { assertInvariants(); return variableColOffsets_.back() - variableColOffsets_[blockStart_]; }
  
  
  /** Block count
   */
  size_t nBlocks() const { assertInvariants(); return variableColOffsets_.size() - 1 - blockStart_; }

  
  /** Access a single block in the underlying matrix with read/write access */
  inline Block operator()(size_t block) {
    return range(block, block+1);
  }

  /** Access a const block view */
  inline const constBlock operator()(size_t block) const {
    return range(block, block+1);
  }

  /** access ranges of blocks at a time */
  inline Block range(size_t startBlock, size_t endBlock) {
    assertInvariants();
    size_t actualStartBlock = startBlock + blockStart_;
    size_t actualEndBlock = endBlock + blockStart_;
    checkBlock(actualStartBlock);
    assert(actualEndBlock < variableColOffsets_.size());
    const size_t& startCol = variableColOffsets_[actualStartBlock];
    const size_t& endCol = variableColOffsets_[actualEndBlock];
    return matrix_.block(rowStart_, startCol, rowEnd_-rowStart_, endCol-startCol);
  }

  inline const constBlock range(size_t startBlock, size_t endBlock) const {
    assertInvariants();
    size_t actualStartBlock = startBlock + blockStart_;
    size_t actualEndBlock = endBlock + blockStart_;
    checkBlock(actualStartBlock);
    assert(actualEndBlock < variableColOffsets_.size());
    const size_t& startCol = variableColOffsets_[actualStartBlock];
    const size_t& endCol = variableColOffsets_[actualEndBlock];
    return ((const FullMatrix&)matrix_).block(rowStart_, startCol, rowEnd_-rowStart_, endCol-startCol);
  }

  /** Return the full matrix, *not* including any portions excluded by rowStart(), rowEnd(), and firstBlock() */
  inline Block full() {
    return range(0,nBlocks());
  }

  /** Return the full matrix, *not* including any portions excluded by rowStart(), rowEnd(), and firstBlock() */
  inline const constBlock full() const {
    return range(0,nBlocks());
  }

  /** get a single column out of a block */
  Column column(size_t block, size_t columnOffset) {
    assertInvariants();
    size_t actualBlock = block + blockStart_;
    checkBlock(actualBlock);
    assert(variableColOffsets_[actualBlock] + columnOffset < variableColOffsets_[actualBlock+1]);
    return matrix_.col(variableColOffsets_[actualBlock] + columnOffset).segment(rowStart_, rowEnd_-rowStart_);
  }

  /** get a single column out of a block */
  const constColumn column(size_t block, size_t columnOffset) const {
    assertInvariants();
    size_t actualBlock = block + blockStart_;
    checkBlock(actualBlock);
    assert(variableColOffsets_[actualBlock] + columnOffset < (size_t) matrix_.cols());
    return ((const FullMatrix&)matrix_).col(variableColOffsets_[actualBlock] + columnOffset).segment(rowStart_, rowEnd_-rowStart_);
  }

  size_t offset(size_t block) const {
    assertInvariants();
    size_t actualBlock = block + blockStart_;
    checkBlock(actualBlock);
    return variableColOffsets_[actualBlock];
  }

  /** Get or set the apparent first row of the underlying matrix for all operations */
  size_t& rowStart() { return rowStart_; }

  /** Get or set the apparent last row (exclusive, i.e. rows() == rowEnd() - rowStart()) of the underlying matrix for all operations */
  size_t& rowEnd() { return rowEnd_; }

  /** Get or set the apparent first block for all operations */
  size_t& firstBlock() { return blockStart_; }

  /** Get the apparent first row of the underlying matrix for all operations */
  size_t rowStart() const { return rowStart_; }

  /** Get the apparent last row (exclusive, i.e. rows() == rowEnd() - rowStart()) of the underlying matrix for all operations */
  size_t rowEnd() const { return rowEnd_; }

  /** Get the apparent first block for all operations */
  size_t firstBlock() const { return blockStart_; }

  /** access to full matrix (*including* any portions excluded by rowStart(), rowEnd(), and firstBlock()) */
  const FullMatrix& fullMatrix() const { return matrix_; }

  /**
   * Copy the block structure and resize the underlying matrix, but do not
   * copy the matrix data.  If blockStart(), rowStart(), and/or rowEnd() have
   * been modified, this copies the structure of the corresponding matrix view.
   * In the destination VerticalBlockView, blockStart() and rowStart() will
   * thus be 0, rowEnd() will be cols() of the source VerticalBlockView, and
   * the underlying matrix will be the size of the view of the source matrix.
   */
  template<class RHS>
  void copyStructureFrom(const RHS& rhs) {
    if((size_t) matrix_.rows() != (size_t) rhs.rows() || (size_t) matrix_.cols() != (size_t) rhs.cols())
      matrix_.resize(rhs.rows(), rhs.cols());
    if(rhs.blockStart_ == 0)
      variableColOffsets_ = rhs.variableColOffsets_;
    else {
      variableColOffsets_.resize(rhs.nBlocks() + 1);
      variableColOffsets_[0] = 0;
      size_t j=0;
      assert(rhs.variableColOffsets_.begin()+rhs.blockStart_ < rhs.variableColOffsets_.end()-1);
      for(std::vector<size_t>::const_iterator off=rhs.variableColOffsets_.begin()+rhs.blockStart_; off!=rhs.variableColOffsets_.end()-1; ++off) {
        variableColOffsets_[j+1] = variableColOffsets_[j] + (*(off+1) - *off);
        ++ j;
      }
    }
    rowStart_ = 0;
    rowEnd_ = matrix_.rows();
    blockStart_ = 0;
    assertInvariants();
  }

  /** Copy the block struture and matrix data, resizing the underlying matrix
   * in the process.  This can deal with assigning between different types of
   * underlying matrices, as long as the matrices themselves are assignable.
   * To avoid creating a temporary matrix this assumes no aliasing, i.e. that
   * no part of the underlying matrices refer to the same memory!
   *
   * If blockStart(), rowStart(), and/or rowEnd() have been modified, this
   * copies the structure of the corresponding matrix view.  In the destination
   * VerticalBlockView, blockStart() and rowStart() will thus be 0, rowEnd()
   * will be cols() of the source VerticalBlockView, and the underlying matrix
   * will be the size of the view of the source matrix.
   */
  template<class RHS>
  VerticalBlockView<MATRIX>& assignNoalias(const RHS& rhs) {
    copyStructureFrom(rhs);
    matrix_.noalias() = rhs.full();
    return *this;
  }

  /** Swap the contents of the underlying matrix and the block information with
   * another VerticalBlockView.
   */
  void swap(VerticalBlockView<MATRIX>& other) {
    matrix_.swap(other.matrix_);
    variableColOffsets_.swap(other.variableColOffsets_);
    std::swap(rowStart_, other.rowStart_);
    std::swap(rowEnd_, other.rowEnd_);
    std::swap(blockStart_, other.blockStart_);
    assertInvariants();
    other.assertInvariants();
  }

protected:
  void assertInvariants() const {
    assert((size_t) matrix_.cols() == variableColOffsets_.back());
    assert(blockStart_ < variableColOffsets_.size());
    assert(rowStart_ <= (size_t) matrix_.rows());
    assert(rowEnd_ <= (size_t) matrix_.rows());
    assert(rowStart_ <= rowEnd_);
  }

  void checkBlock(size_t block) const {
    assert((size_t) matrix_.cols() == variableColOffsets_.back());
    assert(block < variableColOffsets_.size()-1);
    assert(variableColOffsets_[block] < (size_t) matrix_.cols() && variableColOffsets_[block+1] <= (size_t) matrix_.cols());
  }

  template<typename ITERATOR>
  void fillOffsets(ITERATOR firstBlockDim, ITERATOR lastBlockDim) {
    variableColOffsets_.resize((lastBlockDim-firstBlockDim)+1);
    variableColOffsets_[0] = 0;
    size_t j=0;
    for(ITERATOR dim=firstBlockDim; dim!=lastBlockDim; ++dim) {
      variableColOffsets_[j+1] = variableColOffsets_[j] + *dim;
      ++ j;
    }
  }

  template<class OTHER> friend class SymmetricBlockView;
  template<class RELATED> friend class VerticalBlockView;

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

/**
 * This class stores a *reference* to a matrix and allows it to be accessed as
 * a collection of blocks.  It also provides for accessing individual
 * columns from those blocks.  When constructed or resized, the caller must
 * provide the dimensions of the blocks, as well as an underlying matrix
 * storage object.  This class will resize the underlying matrix such that it
 * is consistent with the given block dimensions.
 *
 * This class uses a symmetric block structure.  The underlying matrix does not
 * necessarily need to be symmetric.
 *
 * This class also has a parameter that can be changed after construction to
 * change the apparent matrix view.  firstBlock() determines the block that
 * appears to have index 0 for all operations (except re-setting firstBlock()).
 *
 * @addtogroup base
 */
template<class MATRIX>
class SymmetricBlockView {
public:
  typedef MATRIX FullMatrix;
  typedef Eigen::Block<MATRIX> Block;
  typedef Eigen::Block<const MATRIX> constBlock;
  typedef typename FullMatrix::ColXpr::SegmentReturnType Column;
  typedef typename FullMatrix::ConstColXpr::ConstSegmentReturnType constColumn;

private:
  static FullMatrix matrixTemp_; // just for finding types

protected:
  FullMatrix& matrix_; // the reference to the full matrix
  std::vector<size_t> variableColOffsets_; // the starting columns of each block (0-based)

  // Changes apparent matrix view, see main class comment.
  size_t blockStart_; // 0 initially

public:
  /** Construct from an empty matrix (asserts that the matrix is empty) */
  SymmetricBlockView(FullMatrix& matrix) :
    matrix_(matrix), blockStart_(0) {
    fillOffsets((size_t*)0, (size_t*)0);
    assertInvariants();
  }

  /** Construct from iterators over the sizes of each block */
  template<typename ITERATOR>
  SymmetricBlockView(FullMatrix& matrix, ITERATOR firstBlockDim, ITERATOR lastBlockDim) :
  matrix_(matrix), blockStart_(0) {
    fillOffsets(firstBlockDim, lastBlockDim);
    assertInvariants();
  }

  /**
   * Modify the size and structure of the underlying matrix and this block
   * view.  If 'preserve' is true, the underlying matrix data will be copied if
   * the matrix size changes, otherwise the new data will be uninitialized.
   */
  template<typename ITERATOR>
  void resize(ITERATOR firstBlockDim, ITERATOR lastBlockDim, bool preserve) {
    blockStart_ = 0;
    fillOffsets(firstBlockDim, lastBlockDim);
    if (preserve)
    	matrix_.conservativeResize(variableColOffsets_.back(), variableColOffsets_.back());
    else
    	matrix_.resize(variableColOffsets_.back(), variableColOffsets_.back());
  }

  /** Row size
   */
  size_t rows() const { assertInvariants(); return variableColOffsets_.back() - variableColOffsets_[blockStart_]; }

  /** Column size
   */
  size_t cols() const { return rows(); }


  /** Block count
   */
  size_t nBlocks() const { assertInvariants(); return variableColOffsets_.size() - 1 - blockStart_; }


  Block operator()(size_t i_block, size_t j_block) {
    return range(i_block, i_block+1, j_block, j_block+1);
  }

  constBlock operator()(size_t i_block, size_t j_block) const {
    return range(i_block, i_block+1, j_block, j_block+1);
  }

  Block range(size_t i_startBlock, size_t i_endBlock, size_t j_startBlock, size_t j_endBlock) {
    assertInvariants();
    size_t i_actualStartBlock = i_startBlock + blockStart_;
    size_t i_actualEndBlock = i_endBlock + blockStart_;
    size_t j_actualStartBlock = j_startBlock + blockStart_;
    size_t j_actualEndBlock = j_endBlock + blockStart_;
    checkBlock(i_actualStartBlock);
    checkBlock(j_actualStartBlock);
    assert(i_actualEndBlock < variableColOffsets_.size());
    assert(j_actualEndBlock < variableColOffsets_.size());
    return matrix_.block(
    		variableColOffsets_[i_actualStartBlock], variableColOffsets_[j_actualStartBlock],
    		variableColOffsets_[i_actualEndBlock]-variableColOffsets_[i_actualStartBlock],
    		variableColOffsets_[j_actualEndBlock]-variableColOffsets_[j_actualStartBlock]);
  }

  constBlock range(size_t i_startBlock, size_t i_endBlock, size_t j_startBlock, size_t j_endBlock) const {
    assertInvariants();
    size_t i_actualStartBlock = i_startBlock + blockStart_;
    size_t i_actualEndBlock = i_endBlock + blockStart_;
    size_t j_actualStartBlock = j_startBlock + blockStart_;
    size_t j_actualEndBlock = j_endBlock + blockStart_;
    checkBlock(i_actualStartBlock);
    checkBlock(j_actualStartBlock);
    assert(i_actualEndBlock < variableColOffsets_.size());
    assert(j_actualEndBlock < variableColOffsets_.size());
    return ((const FullMatrix&)matrix_).block(
    		variableColOffsets_[i_actualStartBlock], variableColOffsets_[j_actualStartBlock],
    		variableColOffsets_[i_actualEndBlock]-variableColOffsets_[i_actualStartBlock],
    		variableColOffsets_[j_actualEndBlock]-variableColOffsets_[j_actualStartBlock]);
  }

  Block full() {
    return range(0,nBlocks(), 0,nBlocks());
  }

  constBlock full() const {
    return range(0,nBlocks(), 0,nBlocks());
  }

  /** access to full matrix */
  const FullMatrix& fullMatrix() const { return matrix_; }

  Column column(size_t i_block, size_t j_block, size_t columnOffset) {
  	assertInvariants();
  	size_t i_actualBlock = i_block + blockStart_;
  	size_t j_actualBlock = j_block + blockStart_;
  	checkBlock(i_actualBlock);
  	checkBlock(j_actualBlock);
  	assert(i_actualBlock < variableColOffsets_.size());
  	assert(j_actualBlock < variableColOffsets_.size());
  	assert(variableColOffsets_[j_actualBlock] + columnOffset < variableColOffsets_[j_actualBlock+1]);

  	return matrix_.col(
  			variableColOffsets_[j_actualBlock] + columnOffset).segment(
  					variableColOffsets_[i_actualBlock],
  					variableColOffsets_[i_actualBlock+1]-variableColOffsets_[i_actualBlock]);
  }

  constColumn column(size_t i_block, size_t j_block, size_t columnOffset) const {
    assertInvariants();
    size_t i_actualBlock = i_block + blockStart_;
    size_t j_actualBlock = j_block + blockStart_;
    checkBlock(i_actualBlock);
    checkBlock(j_actualBlock);
    assert(i_actualBlock < variableColOffsets_.size());
    assert(j_actualBlock < variableColOffsets_.size());
    assert(variableColOffsets_[j_actualBlock] + columnOffset < variableColOffsets_[j_actualBlock+1]);

    return ((const FullMatrix&)matrix_).col(
        variableColOffsets_[j_actualBlock] + columnOffset).segment(
            variableColOffsets_[i_actualBlock],
            variableColOffsets_[i_actualBlock+1]-variableColOffsets_[i_actualBlock]);
//    assertInvariants();
//    size_t j_actualBlock = j_block + blockStart_;
//    assert(variableColOffsets_[j_actualBlock] + columnOffset < variableColOffsets_[j_actualBlock+1]);
//    constBlock blockMat(operator()(i_block, j_block));
//    return constColumn(blockMat, columnOffset);
  }

  Column rangeColumn(size_t i_startBlock, size_t i_endBlock, size_t j_block, size_t columnOffset) {
    assertInvariants();

    size_t i_actualStartBlock = i_startBlock + blockStart_;
    size_t i_actualEndBlock = i_endBlock + blockStart_;
    size_t j_actualStartBlock = j_block + blockStart_;
    checkBlock(i_actualStartBlock);
    checkBlock(j_actualStartBlock);
    assert(i_actualEndBlock < variableColOffsets_.size());
    assert(variableColOffsets_[j_actualStartBlock] + columnOffset < variableColOffsets_[j_actualStartBlock+1]);

    return matrix_.col(
    		variableColOffsets_[j_actualStartBlock] + columnOffset).segment(
    				variableColOffsets_[i_actualStartBlock],
    				variableColOffsets_[i_actualEndBlock]-variableColOffsets_[i_actualStartBlock]);
  }

  constColumn rangeColumn(size_t i_startBlock, size_t i_endBlock, size_t j_block, size_t columnOffset) const {
    assertInvariants();

    size_t i_actualStartBlock = i_startBlock + blockStart_;
    size_t i_actualEndBlock = i_endBlock + blockStart_;
    size_t j_actualStartBlock = j_block + blockStart_;
    checkBlock(i_actualStartBlock);
    checkBlock(j_actualStartBlock);
    assert(i_actualEndBlock < variableColOffsets_.size());
    assert(variableColOffsets_[j_actualStartBlock] + columnOffset < variableColOffsets_[j_actualStartBlock+1]);

    return ((const FullMatrix&)matrix_).col(
    		variableColOffsets_[j_actualStartBlock] + columnOffset).segment(
    				variableColOffsets_[i_actualStartBlock],
    				variableColOffsets_[i_actualEndBlock]-variableColOffsets_[i_actualStartBlock]);
  }

  size_t offset(size_t block) const {
    assertInvariants();
    size_t actualBlock = block + blockStart_;
    checkBlock(actualBlock);
    return variableColOffsets_[actualBlock];
  }

  size_t& blockStart() { return blockStart_; }
  size_t blockStart() const { return blockStart_; }

  /** Copy the block structure and resize the underlying matrix, but do not
   * copy the matrix data.  If blockStart() has been modified, this copies the
   * structure of the corresponding matrix view.  In the destination
   * SymmetricBlockView, startBlock() will thus be 0 and the underlying matrix
   * will be the size of the view of the source matrix.
   */
  template<class RHS>
  void copyStructureFrom(const RHS& rhs) {
    matrix_.resize(rhs.cols(), rhs.cols());
    if(rhs.blockStart_ == 0)
      variableColOffsets_ = rhs.variableColOffsets_;
    else {
      variableColOffsets_.resize(rhs.nBlocks() + 1);
      variableColOffsets_[0] = 0;
      size_t j=0;
      assert(rhs.variableColOffsets_.begin()+rhs.blockStart_ < rhs.variableColOffsets_.end()-1);
      for(std::vector<size_t>::const_iterator off=rhs.variableColOffsets_.begin()+rhs.blockStart_; off!=rhs.variableColOffsets_.end()-1; ++off) {
        variableColOffsets_[j+1] = variableColOffsets_[j] + (*(off+1) - *off);
        ++ j;
      }
    }
    blockStart_ = 0;
    assertInvariants();
  }

  /** Copy the block struture and matrix data, resizing the underlying matrix
   * in the process.  This can deal with assigning between different types of
   * underlying matrices, as long as the matrices themselves are assignable.
   * To avoid creating a temporary matrix this assumes no aliasing, i.e. that
   * no part of the underlying matrices refer to the same memory!
   *
   * If blockStart() has been modified, this copies the structure of the
   * corresponding matrix view.  In the destination SymmetricBlockView,
   * startBlock() will thus be 0 and the underlying matrix will be the size
   * of the view of the source matrix.
   */
  template<class RHSMATRIX>
  SymmetricBlockView<MATRIX>& assignNoalias(const SymmetricBlockView<RHSMATRIX>& rhs) {
    copyStructureFrom(rhs);
    matrix_.noalias() = rhs.full();
    return *this;
  }

  /** Swap the contents of the underlying matrix and the block information with
   * another VerticalBlockView.
   */
  void swap(SymmetricBlockView<MATRIX>& other) {
    matrix_.swap(other.matrix_);
    variableColOffsets_.swap(other.variableColOffsets_);
    std::swap(blockStart_, other.blockStart_);
    assertInvariants();
    other.assertInvariants();
  }

protected:
  void assertInvariants() const {
    assert(matrix_.rows() == matrix_.cols());
    assert((size_t) matrix_.cols() == variableColOffsets_.back());
    assert(blockStart_ < variableColOffsets_.size());
  }

  void checkBlock(size_t block) const {
    assert(matrix_.rows() == matrix_.cols());
    assert((size_t) matrix_.cols() == variableColOffsets_.back());
    assert(block < variableColOffsets_.size()-1);
    assert(variableColOffsets_[block] < (size_t) matrix_.cols() && variableColOffsets_[block+1] <= (size_t) matrix_.cols());
  }

  template<typename ITERATOR>
  void fillOffsets(ITERATOR firstBlockDim, ITERATOR lastBlockDim) {
    variableColOffsets_.resize((lastBlockDim-firstBlockDim)+1);
    variableColOffsets_[0] = 0;
    size_t j=0;
    for(ITERATOR dim=firstBlockDim; dim!=lastBlockDim; ++dim) {
      variableColOffsets_[j+1] = variableColOffsets_[j] + *dim;
      ++ j;
    }
  }

  template<class RELATED> friend class SymmetricBlockView;
  template<class OTHER> friend class VerticalBlockView;

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

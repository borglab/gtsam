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
 * @created Sep 18, 2010
 */
#pragma once

#include <vector>
#include <boost/numeric/ublas/matrix_proxy.hpp>

namespace gtsam {

/** This is a wrapper around ublas::matrix_column that stores a copy of a
 * ublas::matrix_range.  This does not copy the matrix data itself.  The
 * purpose of this class is to be allow a column-of-a-range to be returned
 * from a function, given that a standard column-of-a-range just stores a
 * reference to the range.  The stored range stores a reference to the original
 * matrix.
 */
template<class MATRIX>
class BlockColumn : public boost::numeric::ublas::vector_expression<BlockColumn<MATRIX> > {
protected:
  typedef boost::numeric::ublas::matrix_range<MATRIX> Range;
  typedef boost::numeric::ublas::matrix_column<Range> Base;
  Range range_;
  Base base_;
public:
  typedef BlockColumn<MATRIX> Self;
  typedef typename Base::matrix_type matrix_type;
  typedef typename Base::size_type size_type;
  typedef typename Base::difference_type difference_type;
  typedef typename Base::value_type value_type;
  typedef typename Base::const_reference const_reference;
  typedef typename Base::reference reference;
  typedef typename Base::storage_category storage_category;
  typedef Self closure_type;
  typedef const Self const_closure_type;
  typedef typename Base::iterator iterator;
  typedef typename Base::const_iterator const_iterator;

  BlockColumn(const boost::numeric::ublas::matrix_range<MATRIX>& block, size_t column) :
    range_(block), base_(range_, column) {}
  BlockColumn(const BlockColumn& rhs) :
    range_(rhs.range_), base_(rhs.base_) {}
  BlockColumn& operator=(const BlockColumn& rhs) { base_.operator=(rhs.base_); return *this; }
  template<class AE> BlockColumn& operator=(const boost::numeric::ublas::vector_expression<AE>& rhs) { base_.operator=(rhs); return *this; }
  typename Base::size_type size() const { return base_.size(); }
  const typename Base::matrix_closure_type& data() const { return base_.data(); }
  typename Base::matrix_closure_type& data() { return base_.data(); }
  typename Base::const_reference operator()(typename Base::size_type i) const { return base_(i); }
  typename Base::reference operator()(typename Base::size_type i) { return base_(i); }
  BlockColumn& assign_temporary(BlockColumn& rhs) { base_.assign_temporary(rhs.base_); return *this; }
  BlockColumn& assign_temporary(Base& rhs) { base_.assign_temporary(rhs); return *this; }
  bool same_closure(const BlockColumn& rhs) { return base_.same_closure(rhs.base_); }
  bool same_closure(const Base& rhs) { return base_.same_closure(rhs); }
  template<class AE> BlockColumn& assign(const boost::numeric::ublas::vector_expression<AE>& rhs) { base_.assign(rhs); return *this; }
  iterator begin() { return base_.begin(); }
  const_iterator begin() const { return base_.begin(); }
  iterator end() { return base_.end(); }
  const_iterator end() const { return base_.end(); }
};

/**
 * This class stores a *reference* to a matrix and allows it to be accessed as
 * a collection of vertical blocks.  It also provides for accessing individual
 * columns from those blocks.  When constructed or resized, the caller must
 * provide the dimensions of the blocks, as well as an underlying matrix
 * storage object.  This class will resize the underlying matrix such that it
 * is consistent with the given block dimensions.
 *
 * This class also has three parameters that can be changed after construction
 * that change the
 */
template<class MATRIX>
class VerticalBlockView {
public:
  typedef MATRIX FullMatrix;
  typedef typename boost::numeric::ublas::matrix_range<MATRIX> Block;
  typedef typename boost::numeric::ublas::matrix_range<const MATRIX> constBlock;
  typedef BlockColumn<MATRIX> Column;
  typedef BlockColumn<const MATRIX> constColumn;

protected:
  FullMatrix& matrix_;
  std::vector<size_t> variableColOffsets_;
  size_t rowStart_;
  size_t rowEnd_;
  size_t blockStart_;

public:
  /** Construct from an empty matrix (asserts that the matrix is empty) */
  VerticalBlockView(FullMatrix& matrix);

  /** Construct from iterators over the sizes of each vertical block */
  template<typename ITERATOR>
  VerticalBlockView(FullMatrix& matrix, ITERATOR firstBlockDim, ITERATOR lastBlockDim);

  /** Construct from a vector of the sizes of each vertical block, resize the
   * matrix so that its height is matrixNewHeight and its width fits the given
   * block dimensions.
   */
  template<typename ITERATOR>
  VerticalBlockView(FullMatrix& matrix, ITERATOR firstBlockDim, ITERATOR lastBlockDim, size_t matrixNewHeight);
  
  /** Row size 
   */
  size_t size1() const { assertInvariants(); return rowEnd_ - rowStart_; }
  
  /** Column size
   */ 
  size_t size2() const { assertInvariants(); return variableColOffsets_.back() - variableColOffsets_[blockStart_]; }
  
  
  /** Block count
   */
  size_t nBlocks() const { assertInvariants(); return variableColOffsets_.size() - 1 - blockStart_; }

  
  Block operator()(size_t block) {
    return range(block, block+1);
  }

  constBlock operator()(size_t block) const {
    return range(block, block+1);
  }

  Block range(size_t startBlock, size_t endBlock) {
    assertInvariants();
    size_t actualStartBlock = startBlock + blockStart_;
    size_t actualEndBlock = endBlock + blockStart_;
    checkBlock(actualStartBlock);
    assert(actualEndBlock < variableColOffsets_.size());
    return Block(matrix_,
        boost::numeric::ublas::range(rowStart_, rowEnd_),
        boost::numeric::ublas::range(variableColOffsets_[actualStartBlock], variableColOffsets_[actualEndBlock]));
  }

  constBlock range(size_t startBlock, size_t endBlock) const {
    assertInvariants();
    size_t actualStartBlock = startBlock + blockStart_;
    size_t actualEndBlock = endBlock + blockStart_;
    checkBlock(actualStartBlock);
    assert(actualEndBlock < variableColOffsets_.size());
    return constBlock(matrix_,
        boost::numeric::ublas::range(rowStart_, rowEnd_),
        boost::numeric::ublas::range(variableColOffsets_[actualStartBlock], variableColOffsets_[actualEndBlock]));
  }

  Column column(size_t block, size_t columnOffset) {
    assertInvariants();
    size_t actualBlock = block + blockStart_;
    checkBlock(actualBlock);
    assert(variableColOffsets_[actualBlock] + columnOffset < matrix_.size2());
    Block blockMat(operator()(block));
    return Column(blockMat, columnOffset);
  }

  constColumn column(size_t block, size_t columnOffset) const {
    assertInvariants();
    size_t actualBlock = block + blockStart_;
    checkBlock(actualBlock);
    assert(variableColOffsets_[actualBlock] + columnOffset < matrix_.size2());
    constBlock blockMat(operator()(block));
    return constColumn(blockMat, columnOffset);
  }

  size_t offset(size_t block) const {
    assertInvariants();
    size_t actualBlock = block + blockStart_;
    checkBlock(actualBlock);
    return variableColOffsets_[actualBlock] - variableColOffsets_[blockStart_];
  }

  size_t& rowStart() { return rowStart_; }
  size_t& rowEnd() { return rowEnd_; }
  size_t& firstBlock() { return blockStart_; }
  size_t rowStart() const { return rowStart_; }
  size_t rowEnd() const { return rowEnd_; }
  size_t firstBlock() const { return blockStart_; }

  /** Copy the block structure and resize the underlying matrix, but do not
   * copy the matrix data.
   */
  template<class RHSMATRIX>
  void copyStructureFrom(const VerticalBlockView<RHSMATRIX>& rhs);

  /** Copy the block struture and matrix data, resizing the underlying matrix
   * in the process.  This can deal with assigning between different types of
   * underlying matrices, as long as the matrices themselves are assignable.
   * To avoid creating a temporary matrix this assumes no aliasing, i.e. that
   * no part of the underlying matrices refer to the same memory!
   */
  template<class RHSMATRIX>
  VerticalBlockView<MATRIX>& assignNoalias(const VerticalBlockView<RHSMATRIX>& rhs);

  /** Swap the contents of the underlying matrix and the block information with
   * another VerticalBlockView.
   */
  void swap(VerticalBlockView<MATRIX>& other);

protected:
  void assertInvariants() const {
    assert(matrix_.size2() == variableColOffsets_.back());
    assert(blockStart_ < variableColOffsets_.size());
    assert(rowStart_ <= matrix_.size1());
    assert(rowEnd_ <= matrix_.size1());
    assert(rowStart_ <= rowEnd_);
  }

  void checkBlock(size_t block) const {
    assert(matrix_.size2() == variableColOffsets_.back());
    assert(block < variableColOffsets_.size()-1);
    assert(variableColOffsets_[block] < matrix_.size2() && variableColOffsets_[block+1] <= matrix_.size2());
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

  template<class RHSMATRIX>
  friend class VerticalBlockView<MATRIX>;
};

/* ************************************************************************* */
template<class MATRIX>
VerticalBlockView<MATRIX>::VerticalBlockView(FullMatrix& matrix) :
matrix_(matrix), rowStart_(0), rowEnd_(matrix_.size1()), blockStart_(0) {
  fillOffsets((size_t*)0, (size_t*)0);
  assertInvariants();
}

/* ************************************************************************* */
template<class MATRIX>
template<typename ITERATOR>
VerticalBlockView<MATRIX>::VerticalBlockView(FullMatrix& matrix, ITERATOR firstBlockDim, ITERATOR lastBlockDim) :
matrix_(matrix), rowStart_(0), rowEnd_(matrix_.size1()), blockStart_(0) {
  fillOffsets(firstBlockDim, lastBlockDim);
  assertInvariants();
}

/* ************************************************************************* */
template<class MATRIX>
template<typename ITERATOR>
VerticalBlockView<MATRIX>::VerticalBlockView(FullMatrix& matrix, ITERATOR firstBlockDim, ITERATOR lastBlockDim, size_t matrixNewHeight) :
matrix_(matrix), rowStart_(0), rowEnd_(matrixNewHeight), blockStart_(0) {
  fillOffsets(firstBlockDim, lastBlockDim);
  matrix_.resize(matrixNewHeight, variableColOffsets_.back(), false);
  assertInvariants();
}

/* ************************************************************************* */
template<class MATRIX>
template<class RHSMATRIX>
void VerticalBlockView<MATRIX>::copyStructureFrom(const VerticalBlockView<RHSMATRIX>& rhs) {
  matrix_.resize(rhs.rowEnd() - rhs.rowStart(), rhs.range(0, rhs.nBlocks()).size2(), false);
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
  rowEnd_ = matrix_.size1();
  blockStart_ = 0;
  assertInvariants();
}

/* ************************************************************************* */
template<class MATRIX>
template<class RHSMATRIX>
VerticalBlockView<MATRIX>& VerticalBlockView<MATRIX>::assignNoalias(const VerticalBlockView<RHSMATRIX>& rhs) {
  copyStructureFrom(rhs);
  boost::numeric::ublas::noalias(matrix_) = rhs.range(0, rhs.nBlocks());
  return *this;
}

/* ************************************************************************* */
template<class MATRIX>
void VerticalBlockView<MATRIX>::swap(VerticalBlockView<MATRIX>& other) {
  matrix_.swap(other.matrix_);
  variableColOffsets_.swap(other.variableColOffsets_);
  std::swap(rowStart_, other.rowStart_);
  std::swap(rowEnd_, other.rowEnd_);
  std::swap(blockStart_, other.blockStart_);
  assertInvariants();
  other.assertInvariants();
}


}

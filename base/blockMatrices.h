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
template<class Matrix>
class BlockColumn : public boost::numeric::ublas::vector_expression<BlockColumn<Matrix> > {
protected:
  typedef boost::numeric::ublas::matrix_range<Matrix> Range;
  typedef boost::numeric::ublas::matrix_column<Range> Base;
  Range range_;
  Base base_;
public:
  typedef BlockColumn<Matrix> Self;
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

  BlockColumn(const boost::numeric::ublas::matrix_range<Matrix>& block, size_t column) :
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
template<class Matrix>
class VerticalBlockView {
public:
  typedef Matrix matrix_type;
  typedef typename boost::numeric::ublas::matrix_range<Matrix> block_type;
  typedef typename boost::numeric::ublas::matrix_range<const Matrix> const_block_type;
  typedef BlockColumn<Matrix> column_type;
  typedef BlockColumn<const Matrix> const_column_type;

protected:
  matrix_type& matrix_;
  std::vector<size_t> variableColOffsets_;
  size_t rowStart_;
  size_t rowEnd_;
  size_t blockStart_;

public:
  /** Construct from an empty matrix (asserts that the matrix is empty) */
  VerticalBlockView(matrix_type& matrix);

  /** Construct from iterators over the sizes of each vertical block */
  template<typename Iterator>
  VerticalBlockView(matrix_type& matrix, Iterator firstBlockDim, Iterator lastBlockDim);

  /** Construct from a vector of the sizes of each vertical block, resize the
   * matrix so that its height is matrixNewHeight and its width fits the given
   * block dimensions.
   */
  template<typename Iterator>
  VerticalBlockView(matrix_type& matrix, Iterator firstBlockDim, Iterator lastBlockDim, size_t matrixNewHeight);

  size_t size1() const { checkInvariants(); return rowEnd_ - rowStart_; }
  size_t size2() const { checkInvariants(); return variableColOffsets_.back() - variableColOffsets_[blockStart_]; }
  size_t nBlocks() const { checkInvariants(); return variableColOffsets_.size() - 1 - blockStart_; }

  block_type operator()(size_t block) {
    return range(block, block+1);
  }

  const_block_type operator()(size_t block) const {
    return range(block, block+1);
  }

  block_type range(size_t startBlock, size_t endBlock) {
    checkInvariants();
    size_t actualStartBlock = startBlock + blockStart_;
    size_t actualEndBlock = endBlock + blockStart_;
    checkBlock(actualStartBlock);
    assert(actualEndBlock < variableColOffsets_.size());
    return block_type(matrix_,
        boost::numeric::ublas::range(rowStart_, rowEnd_),
        boost::numeric::ublas::range(variableColOffsets_[actualStartBlock], variableColOffsets_[actualEndBlock]));
  }

  const_block_type range(size_t startBlock, size_t endBlock) const {
    checkInvariants();
    size_t actualStartBlock = startBlock + blockStart_;
    size_t actualEndBlock = endBlock + blockStart_;
    checkBlock(actualStartBlock);
    assert(actualEndBlock < variableColOffsets_.size());
    return const_block_type(matrix_,
        boost::numeric::ublas::range(rowStart_, rowEnd_),
        boost::numeric::ublas::range(variableColOffsets_[actualStartBlock], variableColOffsets_[actualEndBlock]));
  }

  column_type column(size_t block, size_t columnOffset) {
    checkInvariants();
    size_t actualBlock = block + blockStart_;
    checkBlock(actualBlock);
    assert(variableColOffsets_[actualBlock] + columnOffset < matrix_.size2());
    block_type blockMat(operator()(block));
    return column_type(blockMat, columnOffset);
  }

  const_column_type column(size_t block, size_t columnOffset) const {
    checkInvariants();
    size_t actualBlock = block + blockStart_;
    checkBlock(actualBlock);
    assert(variableColOffsets_[actualBlock] + columnOffset < matrix_.size2());
    const_block_type blockMat(operator()(block));
    return const_column_type(blockMat, columnOffset);
  }

  size_t offset(size_t block) const {
    checkInvariants();
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
  template<class RhsMatrix>
  void copyStructureFrom(const VerticalBlockView<RhsMatrix>& rhs);

  /** Copy the block struture and matrix data, resizing the underlying matrix
   * in the process.  This can deal with assigning between different types of
   * underlying matrices, as long as the matrices themselves are assignable.
   * To avoid creating a temporary matrix this assumes no aliasing, i.e. that
   * no part of the underlying matrices refer to the same memory!
   */
  template<class RhsMatrix>
  VerticalBlockView<Matrix>& assignNoalias(const VerticalBlockView<RhsMatrix>& rhs);

protected:
  void checkInvariants() const {
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

  template<typename Iterator>
  void fillOffsets(Iterator firstBlockDim, Iterator lastBlockDim) {
    variableColOffsets_.resize((lastBlockDim-firstBlockDim)+1);
    variableColOffsets_[0] = 0;
    size_t j=0;
    for(Iterator dim=firstBlockDim; dim!=lastBlockDim; ++dim) {
      variableColOffsets_[j+1] = variableColOffsets_[j] + *dim;
      ++ j;
    }
  }

  template<class RhsMatrix>
  friend class VerticalBlockView<Matrix>;
};

template<class Matrix>
VerticalBlockView<Matrix>::VerticalBlockView(matrix_type& matrix) :
matrix_(matrix), rowStart_(0), rowEnd_(matrix_.size1()), blockStart_(0) {
  fillOffsets((size_t*)0, (size_t*)0);
  checkInvariants();
}

template<class Matrix>
template<typename Iterator>
VerticalBlockView<Matrix>::VerticalBlockView(matrix_type& matrix, Iterator firstBlockDim, Iterator lastBlockDim) :
matrix_(matrix), rowStart_(0), rowEnd_(matrix_.size1()), blockStart_(0) {
  fillOffsets(firstBlockDim, lastBlockDim);
  checkInvariants();
}

template<class Matrix>
template<typename Iterator>
VerticalBlockView<Matrix>::VerticalBlockView(matrix_type& matrix, Iterator firstBlockDim, Iterator lastBlockDim, size_t matrixNewHeight) :
matrix_(matrix), rowStart_(0), rowEnd_(matrixNewHeight), blockStart_(0) {
  fillOffsets(firstBlockDim, lastBlockDim);
  matrix_.resize(matrixNewHeight, variableColOffsets_.back(), false);
  checkInvariants();
}

template<class Matrix>
template<class RhsMatrix>
void VerticalBlockView<Matrix>::copyStructureFrom(const VerticalBlockView<RhsMatrix>& rhs) {
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
  checkInvariants();
}

template<class Matrix>
template<class RhsMatrix>
VerticalBlockView<Matrix>& VerticalBlockView<Matrix>::assignNoalias(const VerticalBlockView<RhsMatrix>& rhs) {
  copyStructureFrom(rhs);
  boost::numeric::ublas::noalias(matrix_) = rhs.range(0, rhs.nBlocks());
  return *this;
}


}

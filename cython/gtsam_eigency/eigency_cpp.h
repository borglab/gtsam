#include <Eigen/Dense>

#include <iostream>
#include <stdexcept>
#include <complex>

typedef ::std::complex< double > __pyx_t_double_complex;
typedef ::std::complex< float > __pyx_t_float_complex;

#include "conversions_api.h"

#ifndef EIGENCY_CPP
#define EIGENCY_CPP

namespace eigency {

template<typename Scalar>
inline PyArrayObject *_ndarray_view(Scalar *, long rows, long cols, bool is_row_major, long outer_stride=0, long inner_stride=0);
template<typename Scalar>
inline PyArrayObject *_ndarray_copy(const Scalar *, long rows, long cols, bool is_row_major, long outer_stride=0, long inner_stride=0);

// Strides:
// Eigen and numpy differ in their way of dealing with strides. Eigen has the concept of outer and
// inner strides, which are dependent on whether the array/matrix is row-major of column-major:
//     Inner stride: denotes the offset between succeeding elements in each row (row-major) or column (column-major).
//     Outer stride: denotes the offset between succeeding rows (row-major) or succeeding columns (column-major).
// In contrast, numpy's stride is simply a measure of how fast each dimension should be incremented.
// Consequently, a switch in numpy storage order from row-major to column-major involves a switch
// in strides, while it does not affect the stride in Eigen.
template<>
inline PyArrayObject *_ndarray_view<double>(double *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major) {
        // Eigen row-major mode: row_stride=outer_stride, and col_stride=inner_stride
        // If no stride is given, the row_stride is set to the number of columns.
        return ndarray_double_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    } else {
        // Eigen column-major mode: row_stride=outer_stride, and col_stride=inner_stride
        // If no stride is given, the cow_stride is set to the number of rows.        
        return ndarray_double_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
    }
}
template<>
inline PyArrayObject *_ndarray_copy<double>(const double *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_double_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_double_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<float>(float *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_float_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_float_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<float>(const float *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_float_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_float_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<long>(long *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_long_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_long_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<long>(const long *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_long_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_long_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<unsigned long>(unsigned long *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_ulong_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_ulong_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<unsigned long>(const unsigned long *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_ulong_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_ulong_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<int>(int *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_int_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_int_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<int>(const int *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_int_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_int_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<unsigned int>(unsigned int *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_uint_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_uint_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<unsigned int>(const unsigned int *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_uint_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_uint_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<short>(short *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_short_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_short_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<short>(const short *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_short_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_short_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<unsigned short>(unsigned short *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_ushort_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_ushort_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<unsigned short>(const unsigned short *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_ushort_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_ushort_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<signed char>(signed char *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_schar_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_schar_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<signed char>(const signed char *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_schar_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_schar_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<unsigned char>(unsigned char *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_uchar_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_uchar_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<unsigned char>(const unsigned char *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_uchar_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_uchar_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<std::complex<double> >(std::complex<double> *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_complex_double_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_complex_double_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<std::complex<double> >(const std::complex<double> *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_complex_double_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_complex_double_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}

template<>
inline PyArrayObject *_ndarray_view<std::complex<float> >(std::complex<float> *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_complex_float_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_complex_float_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}
template<>
inline PyArrayObject *_ndarray_copy<std::complex<float> >(const std::complex<float> *data, long rows, long cols, bool is_row_major, long outer_stride, long inner_stride) {
    if (is_row_major)
        return ndarray_copy_complex_float_C(data, rows, cols, outer_stride>0?outer_stride:cols, inner_stride>0?inner_stride:1);
    else
        return ndarray_copy_complex_float_F(data, rows, cols, inner_stride>0?inner_stride:1, outer_stride>0?outer_stride:rows);
}


template <typename Derived>
inline PyArrayObject *ndarray(Eigen::PlainObjectBase<Derived> &m) {
    import_gtsam_eigency__conversions();
    return _ndarray_view(m.data(), m.rows(), m.cols(), m.IsRowMajor);
}
// If C++11 is available, check if m is an r-value reference, in
// which case a copy should always be made
#if __cplusplus >= 201103L
template <typename Derived>
inline PyArrayObject *ndarray(Eigen::PlainObjectBase<Derived> &&m) {
    import_gtsam_eigency__conversions();
    return _ndarray_copy(m.data(), m.rows(), m.cols(), m.IsRowMajor);
}
#endif
template <typename Derived>
inline PyArrayObject *ndarray(const Eigen::PlainObjectBase<Derived> &m) {
    import_gtsam_eigency__conversions();
    return _ndarray_copy(m.data(), m.rows(), m.cols(), m.IsRowMajor);
}
template <typename Derived>
inline PyArrayObject *ndarray_view(Eigen::PlainObjectBase<Derived> &m) {
    import_gtsam_eigency__conversions();
    return _ndarray_view(m.data(), m.rows(), m.cols(), m.IsRowMajor);
}
template <typename Derived>
inline PyArrayObject *ndarray_view(const Eigen::PlainObjectBase<Derived> &m) {
    import_gtsam_eigency__conversions();
    return _ndarray_view(const_cast<typename Derived::Scalar*>(m.data()), m.rows(), m.cols(), m.IsRowMajor);
}
template <typename Derived>
inline PyArrayObject *ndarray_copy(const Eigen::PlainObjectBase<Derived> &m) {
    import_gtsam_eigency__conversions();
    return _ndarray_copy(m.data(), m.rows(), m.cols(), m.IsRowMajor);
}

template <typename Derived, int MapOptions, typename Stride>
inline PyArrayObject *ndarray(Eigen::Map<Derived, MapOptions, Stride> &m) {
    import_gtsam_eigency__conversions();
    return _ndarray_view(m.data(), m.rows(), m.cols(), m.IsRowMajor, m.outerStride(), m.innerStride());
}
template <typename Derived, int MapOptions, typename Stride>
inline PyArrayObject *ndarray(const Eigen::Map<Derived, MapOptions, Stride> &m) {
    import_gtsam_eigency__conversions();
    // Since this is a map, we assume that ownership is correctly taken care
    // of, and we avoid taking a copy
    return _ndarray_view(const_cast<typename Derived::Scalar*>(m.data()), m.rows(), m.cols(), m.IsRowMajor, m.outerStride(), m.innerStride());
}
template <typename Derived, int MapOptions, typename Stride>
inline PyArrayObject *ndarray_view(Eigen::Map<Derived, MapOptions, Stride> &m) {
    import_gtsam_eigency__conversions();
    return _ndarray_view(m.data(), m.rows(), m.cols(), m.IsRowMajor, m.outerStride(), m.innerStride());
}
template <typename Derived, int MapOptions, typename Stride>
inline PyArrayObject *ndarray_view(const Eigen::Map<Derived, MapOptions, Stride> &m) {
    import_gtsam_eigency__conversions();
    return _ndarray_view(const_cast<typename Derived::Scalar*>(m.data()), m.rows(), m.cols(), m.IsRowMajor, m.outerStride(), m.innerStride());
}
template <typename Derived, int MapOptions, typename Stride>
inline PyArrayObject *ndarray_copy(const Eigen::Map<Derived, MapOptions, Stride> &m) {
    import_gtsam_eigency__conversions();
    return _ndarray_copy(m.data(), m.rows(), m.cols(), m.IsRowMajor, m.outerStride(), m.innerStride());
}


template <typename MatrixType,
          int _MapOptions = Eigen::Unaligned,
          typename _StrideType=Eigen::Stride<0,0> >
class MapBase: public Eigen::Map<MatrixType, _MapOptions, _StrideType> {
public:
    typedef Eigen::Map<MatrixType, _MapOptions, _StrideType> Base;
    typedef typename Base::Scalar Scalar;

    MapBase(Scalar* data,
            long rows,
            long cols,
            _StrideType stride=_StrideType())
        : Base(data,
               // If both dimensions are dynamic or dimensions match, accept dimensions as they are
               ((Base::RowsAtCompileTime==Eigen::Dynamic && Base::ColsAtCompileTime==Eigen::Dynamic) ||
                (Base::RowsAtCompileTime==rows && Base::ColsAtCompileTime==cols))
               ? rows
               // otherwise, test if swapping them makes them fit
               : ((Base::RowsAtCompileTime==cols || Base::ColsAtCompileTime==rows)
                  ? cols
                  : rows),
               ((Base::RowsAtCompileTime==Eigen::Dynamic && Base::ColsAtCompileTime==Eigen::Dynamic) ||
                (Base::RowsAtCompileTime==rows && Base::ColsAtCompileTime==cols))
               ? cols
               : ((Base::RowsAtCompileTime==cols || Base::ColsAtCompileTime==rows)
                  ? rows
                  : cols),
               stride
            )  {}
};


template <template<class,int,int,int,int,int> class DenseBase,
          typename Scalar,
          int _Rows, int _Cols,
          int _Options = Eigen::AutoAlign |
#if defined(__GNUC__) && __GNUC__==3 && __GNUC_MINOR__==4
    // workaround a bug in at least gcc 3.4.6
    // the innermost ?: ternary operator is misparsed. We write it slightly
    // differently and this makes gcc 3.4.6 happy, but it's ugly.
    // The error would only show up with EIGEN_DEFAULT_TO_ROW_MAJOR is defined
    // (when EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION is RowMajor)
                          ( (_Rows==1 && _Cols!=1) ? Eigen::RowMajor
// EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION contains explicit namespace since Eigen 3.1.19
#if EIGEN_VERSION_AT_LEAST(3,2,90)
                          : !(_Cols==1 && _Rows!=1) ? EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION
#else
                          : !(_Cols==1 && _Rows!=1) ? Eigen::EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION
#endif
                          : ColMajor ),
#else
                          ( (_Rows==1 && _Cols!=1) ? Eigen::RowMajor
                          : (_Cols==1 && _Rows!=1) ? Eigen::ColMajor
// EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION contains explicit namespace since Eigen 3.1.19
#if EIGEN_VERSION_AT_LEAST(3,2,90)
                          : EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION ),
#else
                          : Eigen::EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION ),
#endif
#endif
          int _MapOptions = Eigen::Unaligned,
          int _StrideOuter=0, int _StrideInner=0,
          int _MaxRows = _Rows,
          int _MaxCols = _Cols>
class FlattenedMap: public MapBase<DenseBase<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>, _MapOptions, Eigen::Stride<_StrideOuter, _StrideInner> >  {
public:
    typedef MapBase<DenseBase<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>, _MapOptions, Eigen::Stride<_StrideOuter, _StrideInner> > Base;

    FlattenedMap()
        : Base(NULL, 0, 0) {}
    
    FlattenedMap(Scalar *data, long rows, long cols, long outer_stride=0, long inner_stride=0)
        : Base(data, rows, cols,
               Eigen::Stride<_StrideOuter, _StrideInner>(outer_stride, inner_stride)) {
    }

    FlattenedMap(PyArrayObject *object)
        : Base((Scalar *)((PyArrayObject*)object)->data,        
        // : Base(_from_numpy<Scalar>((PyArrayObject*)object),
               (((PyArrayObject*)object)->nd == 2) ? ((PyArrayObject*)object)->dimensions[0] : 1,
               (((PyArrayObject*)object)->nd == 2) ? ((PyArrayObject*)object)->dimensions[1] : ((PyArrayObject*)object)->dimensions[0],
               Eigen::Stride<_StrideOuter, _StrideInner>(_StrideOuter != Eigen::Dynamic ? _StrideOuter : (((PyArrayObject*)object)->nd == 2) ? ((PyArrayObject*)object)->dimensions[0] : 1,
                                                         _StrideInner != Eigen::Dynamic ? _StrideInner : (((PyArrayObject*)object)->nd == 2) ? ((PyArrayObject*)object)->dimensions[1] : ((PyArrayObject*)object)->dimensions[0])) {

        if (((PyObject*)object != Py_None) && !PyArray_ISONESEGMENT(object))
            throw std::invalid_argument("Numpy array must be a in one contiguous segment to be able to be transferred to a Eigen Map.");
    }
    FlattenedMap &operator=(const FlattenedMap &other) {
        // Replace the memory that we point to (not a memory allocation)
        new (this) FlattenedMap(const_cast<Scalar*>(other.data()),
                                other.rows(),
                                other.cols(),
                                other.outerStride(),
                                other.innerStride());
        return *this;
    }
    
    operator Base() const {
        return static_cast<Base>(*this);
    }

    operator Base&() const {
        return static_cast<Base&>(*this);
    }

    operator DenseBase<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>() const {
        return DenseBase<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>(static_cast<Base>(*this));
    }
};


template <typename MatrixType>
class Map: public MapBase<MatrixType> {
public:
    typedef MapBase<MatrixType> Base;
    typedef typename MatrixType::Scalar Scalar;

    Map()
        : Base(NULL, 0, 0) {
    }
    
    Map(Scalar *data, long rows, long cols)
        : Base(data, rows, cols) {}

    Map(PyArrayObject *object)
        : Base((PyObject*)object == Py_None? NULL: (Scalar *)object->data,
               // ROW: If array is in row-major order, transpose (see README)
               (PyObject*)object == Py_None? 0 :
               (PyArray_IS_C_CONTIGUOUS(object)
                ? ((object->nd == 1)
                   ? 1  // ROW: If 1D row-major numpy array, set to 1 (row vector)
                   : object->dimensions[1])
                : object->dimensions[0]),
               // COLUMN: If array is in row-major order: transpose (see README)
               (PyObject*)object == Py_None? 0 :
               (PyArray_IS_C_CONTIGUOUS(object)
                ? object->dimensions[0]
                : ((object->nd == 1)
                   ? 1  // COLUMN: If 1D col-major numpy array, set to length (column vector)
                   : object->dimensions[1]))) {

        if (((PyObject*)object != Py_None) && !PyArray_ISONESEGMENT(object))
            throw std::invalid_argument("Numpy array must be a in one contiguous segment to be able to be transferred to a Eigen Map.");
    }
    
    Map &operator=(const Map &other) {
        // Replace the memory that we point to (not a memory allocation)
        new (this) Map(const_cast<Scalar*>(other.data()),
                       other.rows(),
                       other.cols());
        return *this;
    }
    
    operator Base() const {
        return static_cast<Base>(*this);
    }

    operator Base&() const {
        return static_cast<Base&>(*this);
    }

    operator MatrixType() const {
        return MatrixType(static_cast<Base>(*this));
    }    
};


}

#endif




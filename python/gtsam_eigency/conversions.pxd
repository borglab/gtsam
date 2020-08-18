cimport numpy as np

cdef api np.ndarray[double, ndim=2] ndarray_double_C(double *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[double, ndim=2] ndarray_double_F(double *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[double, ndim=2] ndarray_copy_double_C(const double *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[double, ndim=2] ndarray_copy_double_F(const double *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[float, ndim=2] ndarray_float_C(float *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[float, ndim=2] ndarray_float_F(float *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[float, ndim=2] ndarray_copy_float_C(const float *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[float, ndim=2] ndarray_copy_float_F(const float *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[long, ndim=2] ndarray_long_C(long *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[long, ndim=2] ndarray_long_F(long *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[long, ndim=2] ndarray_copy_long_C(const long *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[long, ndim=2] ndarray_copy_long_F(const long *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[unsigned long, ndim=2] ndarray_ulong_C(unsigned long *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned long, ndim=2] ndarray_ulong_F(unsigned long *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned long, ndim=2] ndarray_copy_ulong_C(const unsigned long *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned long, ndim=2] ndarray_copy_ulong_F(const unsigned long *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[int, ndim=2] ndarray_int_C(int *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[int, ndim=2] ndarray_int_F(int *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[int, ndim=2] ndarray_copy_int_C(const int *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[int, ndim=2] ndarray_copy_int_F(const int *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[unsigned int, ndim=2] ndarray_uint_C(unsigned int *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned int, ndim=2] ndarray_uint_F(unsigned int *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned int, ndim=2] ndarray_copy_uint_C(const unsigned int *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned int, ndim=2] ndarray_copy_uint_F(const unsigned int *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[short, ndim=2] ndarray_short_C(short *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[short, ndim=2] ndarray_short_F(short *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[short, ndim=2] ndarray_copy_short_C(const short *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[short, ndim=2] ndarray_copy_short_F(const short *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[unsigned short, ndim=2] ndarray_ushort_C(unsigned short *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned short, ndim=2] ndarray_ushort_F(unsigned short *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned short, ndim=2] ndarray_copy_ushort_C(const unsigned short *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned short, ndim=2] ndarray_copy_ushort_F(const unsigned short *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[signed char, ndim=2] ndarray_schar_C(signed char *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[signed char, ndim=2] ndarray_schar_F(signed char *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[signed char, ndim=2] ndarray_copy_schar_C(const signed char *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[signed char, ndim=2] ndarray_copy_schar_F(const signed char *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[unsigned char, ndim=2] ndarray_uchar_C(unsigned char *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned char, ndim=2] ndarray_uchar_F(unsigned char *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned char, ndim=2] ndarray_copy_uchar_C(const unsigned char *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[unsigned char, ndim=2] ndarray_copy_uchar_F(const unsigned char *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[np.complex128_t, ndim=2] ndarray_complex_double_C(np.complex128_t *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[np.complex128_t, ndim=2] ndarray_complex_double_F(np.complex128_t *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[np.complex128_t, ndim=2] ndarray_copy_complex_double_C(const np.complex128_t *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[np.complex128_t, ndim=2] ndarray_copy_complex_double_F(const np.complex128_t *data, long rows, long cols, long outer_stride, long inner_stride)

cdef api np.ndarray[np.complex64_t, ndim=2] ndarray_complex_float_C(np.complex64_t *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[np.complex64_t, ndim=2] ndarray_complex_float_F(np.complex64_t *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[np.complex64_t, ndim=2] ndarray_copy_complex_float_C(const np.complex64_t *data, long rows, long cols, long outer_stride, long inner_stride)
cdef api np.ndarray[np.complex64_t, ndim=2] ndarray_copy_complex_float_F(const np.complex64_t *data, long rows, long cols, long outer_stride, long inner_stride)


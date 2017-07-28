cimport cython
cimport numpy as np

ctypedef signed char schar;
ctypedef unsigned char uchar;

ctypedef fused dtype:
    uchar
    schar
    short
    int
    long
    float
    double

ctypedef fused DenseType:
    Matrix
    Array

ctypedef fused Rows:
    _1
    _2
    _3
    _4
    _5
    _6
    _7
    _8
    _9
    _10
    _11
    _12
    _13
    _14
    _15
    _16
    _17
    _18
    _19
    _20
    _21
    _22
    _23
    _24
    _25
    _26
    _27
    _28
    _29
    _30
    _31
    _32
    Dynamic

ctypedef Rows Cols
ctypedef Rows StrideOuter
ctypedef Rows StrideInner

ctypedef fused DenseTypeShort:
    Vector1i
    Vector2i
    Vector3i
    Vector4i
    VectorXi
    RowVector1i
    RowVector2i
    RowVector3i
    RowVector4i
    RowVectorXi
    Matrix1i
    Matrix2i
    Matrix3i
    Matrix4i
    MatrixXi
    Vector1f
    Vector2f
    Vector3f
    Vector4f
    VectorXf
    RowVector1f
    RowVector2f
    RowVector3f
    RowVector4f
    RowVectorXf
    Matrix1f
    Matrix2f
    Matrix3f
    Matrix4f
    MatrixXf
    Vector1d
    Vector2d
    Vector3d
    Vector4d
    VectorXd
    RowVector1d
    RowVector2d
    RowVector3d
    RowVector4d
    RowVectorXd
    Matrix1d
    Matrix2d
    Matrix3d
    Matrix4d
    MatrixXd
    Vector1cf
    Vector2cf
    Vector3cf
    Vector4cf
    VectorXcf
    RowVector1cf
    RowVector2cf
    RowVector3cf
    RowVector4cf
    RowVectorXcf
    Matrix1cf
    Matrix2cf
    Matrix3cf
    Matrix4cf
    MatrixXcf
    Vector1cd
    Vector2cd
    Vector3cd
    Vector4cd
    VectorXcd
    RowVector1cd
    RowVector2cd
    RowVector3cd
    RowVector4cd
    RowVectorXcd
    Matrix1cd
    Matrix2cd
    Matrix3cd
    Matrix4cd
    MatrixXcd
    Array22i
    Array23i
    Array24i
    Array2Xi
    Array32i
    Array33i
    Array34i
    Array3Xi
    Array42i
    Array43i
    Array44i
    Array4Xi
    ArrayX2i
    ArrayX3i
    ArrayX4i
    ArrayXXi
    Array2i
    Array3i
    Array4i
    ArrayXi
    Array22f
    Array23f
    Array24f
    Array2Xf
    Array32f
    Array33f
    Array34f
    Array3Xf
    Array42f
    Array43f
    Array44f
    Array4Xf
    ArrayX2f
    ArrayX3f
    ArrayX4f
    ArrayXXf
    Array2f
    Array3f
    Array4f
    ArrayXf
    Array22d
    Array23d
    Array24d
    Array2Xd
    Array32d
    Array33d
    Array34d
    Array3Xd
    Array42d
    Array43d
    Array44d
    Array4Xd
    ArrayX2d
    ArrayX3d
    ArrayX4d
    ArrayXXd
    Array2d
    Array3d
    Array4d
    ArrayXd
    Array22cf
    Array23cf
    Array24cf
    Array2Xcf
    Array32cf
    Array33cf
    Array34cf
    Array3Xcf
    Array42cf
    Array43cf
    Array44cf
    Array4Xcf
    ArrayX2cf
    ArrayX3cf
    ArrayX4cf
    ArrayXXcf
    Array2cf
    Array3cf
    Array4cf
    ArrayXcf
    Array22cd
    Array23cd
    Array24cd
    Array2Xcd
    Array32cd
    Array33cd
    Array34cd
    Array3Xcd
    Array42cd
    Array43cd
    Array44cd
    Array4Xcd
    ArrayX2cd
    ArrayX3cd
    ArrayX4cd
    ArrayXXcd
    Array2cd
    Array3cd
    Array4cd
    ArrayXcd

ctypedef fused StorageOrder:
    RowMajor
    ColMajor

ctypedef fused MapOptions:
    Aligned
    Unaligned

cdef extern from "eigency_cpp.h" namespace "eigency":

     cdef cppclass _1 "1":
          pass
          
     cdef cppclass _2 "2":
          pass

     cdef cppclass _3 "3":
          pass
          
     cdef cppclass _4 "4":
          pass

     cdef cppclass _5 "5":
          pass
          
     cdef cppclass _6 "6":
          pass

     cdef cppclass _7 "7":
          pass
          
     cdef cppclass _8 "8":
          pass

     cdef cppclass _9 "9":
          pass
          
     cdef cppclass _10 "10":
          pass

     cdef cppclass _11 "11":
          pass
          
     cdef cppclass _12 "12":
          pass

     cdef cppclass _13 "13":
          pass
          
     cdef cppclass _14 "14":
          pass

     cdef cppclass _15 "15":
          pass
          
     cdef cppclass _16 "16":
          pass

     cdef cppclass _17 "17":
          pass
          
     cdef cppclass _18 "18":
          pass

     cdef cppclass _19 "19":
          pass
          
     cdef cppclass _20 "20":
          pass

     cdef cppclass _21 "21":
          pass
          
     cdef cppclass _22 "22":
          pass

     cdef cppclass _23 "23":
          pass
          
     cdef cppclass _24 "24":
          pass

     cdef cppclass _25 "25":
          pass
          
     cdef cppclass _26 "26":
          pass

     cdef cppclass _27 "27":
          pass
          
     cdef cppclass _28 "28":
          pass

     cdef cppclass _29 "29":
          pass
          
     cdef cppclass _30 "30":
          pass

     cdef cppclass _31 "31":
          pass
          
     cdef cppclass _32 "32":
          pass

     cdef cppclass PlainObjectBase:
          pass

     cdef cppclass Map[DenseTypeShort](PlainObjectBase):
         Map() except +
         Map(np.ndarray array) except +

     cdef cppclass FlattenedMap[DenseType, dtype, Rows, Cols]:
         FlattenedMap() except +
         FlattenedMap(np.ndarray array) except +

     cdef cppclass FlattenedMapWithOrder "eigency::FlattenedMap" [DenseType, dtype, Rows, Cols, StorageOrder]:
         FlattenedMapWithOrder() except +
         FlattenedMapWithOrder(np.ndarray array) except +

     cdef cppclass FlattenedMapWithStride "eigency::FlattenedMap" [DenseType, dtype, Rows, Cols, StorageOrder, MapOptions, StrideOuter, StrideInner]:
         FlattenedMapWithStride() except +
         FlattenedMapWithStride(np.ndarray array) except +

     cdef np.ndarray ndarray_view(PlainObjectBase &)
     cdef np.ndarray ndarray_copy(PlainObjectBase &)
     cdef np.ndarray ndarray(PlainObjectBase &)


cdef extern from "eigency_cpp.h" namespace "Eigen":

     cdef cppclass Dynamic:
          pass

     cdef cppclass RowMajor:
          pass

     cdef cppclass ColMajor:
          pass
          
     cdef cppclass Aligned:
          pass

     cdef cppclass Unaligned:
          pass

     cdef cppclass Matrix(PlainObjectBase):
          pass
          
     cdef cppclass Array(PlainObjectBase):
          pass
          
     cdef cppclass VectorXd(PlainObjectBase):
          pass
          
     cdef cppclass Vector1i(PlainObjectBase):
          pass

     cdef cppclass Vector2i(PlainObjectBase):
          pass

     cdef cppclass Vector3i(PlainObjectBase):
          pass

     cdef cppclass Vector4i(PlainObjectBase):
          pass

     cdef cppclass VectorXi(PlainObjectBase):
          pass

     cdef cppclass RowVector1i(PlainObjectBase):
          pass

     cdef cppclass RowVector2i(PlainObjectBase):
          pass

     cdef cppclass RowVector3i(PlainObjectBase):
          pass

     cdef cppclass RowVector4i(PlainObjectBase):
          pass

     cdef cppclass RowVectorXi(PlainObjectBase):
          pass

     cdef cppclass Matrix1i(PlainObjectBase):
          pass

     cdef cppclass Matrix2i(PlainObjectBase):
          pass

     cdef cppclass Matrix3i(PlainObjectBase):
          pass

     cdef cppclass Matrix4i(PlainObjectBase):
          pass

     cdef cppclass MatrixXi(PlainObjectBase):
          pass

     cdef cppclass Vector1f(PlainObjectBase):
          pass

     cdef cppclass Vector2f(PlainObjectBase):
          pass

     cdef cppclass Vector3f(PlainObjectBase):
          pass

     cdef cppclass Vector4f(PlainObjectBase):
          pass

     cdef cppclass VectorXf(PlainObjectBase):
          pass

     cdef cppclass RowVector1f(PlainObjectBase):
          pass

     cdef cppclass RowVector2f(PlainObjectBase):
          pass

     cdef cppclass RowVector3f(PlainObjectBase):
          pass

     cdef cppclass RowVector4f(PlainObjectBase):
          pass

     cdef cppclass RowVectorXf(PlainObjectBase):
          pass

     cdef cppclass Matrix1f(PlainObjectBase):
          pass

     cdef cppclass Matrix2f(PlainObjectBase):
          pass

     cdef cppclass Matrix3f(PlainObjectBase):
          pass

     cdef cppclass Matrix4f(PlainObjectBase):
          pass

     cdef cppclass MatrixXf(PlainObjectBase):
          pass

     cdef cppclass Vector1d(PlainObjectBase):
          pass

     cdef cppclass Vector2d(PlainObjectBase):
          pass

     cdef cppclass Vector3d(PlainObjectBase):
          pass

     cdef cppclass Vector4d(PlainObjectBase):
          pass

     cdef cppclass VectorXd(PlainObjectBase):
          pass

     cdef cppclass RowVector1d(PlainObjectBase):
          pass

     cdef cppclass RowVector2d(PlainObjectBase):
          pass

     cdef cppclass RowVector3d(PlainObjectBase):
          pass

     cdef cppclass RowVector4d(PlainObjectBase):
          pass

     cdef cppclass RowVectorXd(PlainObjectBase):
          pass

     cdef cppclass Matrix1d(PlainObjectBase):
          pass

     cdef cppclass Matrix2d(PlainObjectBase):
          pass

     cdef cppclass Matrix3d(PlainObjectBase):
          pass

     cdef cppclass Matrix4d(PlainObjectBase):
          pass

     cdef cppclass MatrixXd(PlainObjectBase):
          pass

     cdef cppclass Vector1cf(PlainObjectBase):
          pass

     cdef cppclass Vector2cf(PlainObjectBase):
          pass

     cdef cppclass Vector3cf(PlainObjectBase):
          pass

     cdef cppclass Vector4cf(PlainObjectBase):
          pass

     cdef cppclass VectorXcf(PlainObjectBase):
          pass

     cdef cppclass RowVector1cf(PlainObjectBase):
          pass

     cdef cppclass RowVector2cf(PlainObjectBase):
          pass

     cdef cppclass RowVector3cf(PlainObjectBase):
          pass

     cdef cppclass RowVector4cf(PlainObjectBase):
          pass

     cdef cppclass RowVectorXcf(PlainObjectBase):
          pass

     cdef cppclass Matrix1cf(PlainObjectBase):
          pass

     cdef cppclass Matrix2cf(PlainObjectBase):
          pass

     cdef cppclass Matrix3cf(PlainObjectBase):
          pass

     cdef cppclass Matrix4cf(PlainObjectBase):
          pass

     cdef cppclass MatrixXcf(PlainObjectBase):
          pass

     cdef cppclass Vector1cd(PlainObjectBase):
          pass

     cdef cppclass Vector2cd(PlainObjectBase):
          pass

     cdef cppclass Vector3cd(PlainObjectBase):
          pass

     cdef cppclass Vector4cd(PlainObjectBase):
          pass

     cdef cppclass VectorXcd(PlainObjectBase):
          pass

     cdef cppclass RowVector1cd(PlainObjectBase):
          pass

     cdef cppclass RowVector2cd(PlainObjectBase):
          pass

     cdef cppclass RowVector3cd(PlainObjectBase):
          pass

     cdef cppclass RowVector4cd(PlainObjectBase):
          pass

     cdef cppclass RowVectorXcd(PlainObjectBase):
          pass

     cdef cppclass Matrix1cd(PlainObjectBase):
          pass

     cdef cppclass Matrix2cd(PlainObjectBase):
          pass

     cdef cppclass Matrix3cd(PlainObjectBase):
          pass

     cdef cppclass Matrix4cd(PlainObjectBase):
          pass

     cdef cppclass MatrixXcd(PlainObjectBase):
          pass

     cdef cppclass Array22i(PlainObjectBase):
          pass
          
     cdef cppclass Array23i(PlainObjectBase):
          pass
          
     cdef cppclass Array24i(PlainObjectBase):
          pass
          
     cdef cppclass Array2Xi(PlainObjectBase):
          pass
          
     cdef cppclass Array32i(PlainObjectBase):
          pass
          
     cdef cppclass Array33i(PlainObjectBase):
          pass
          
     cdef cppclass Array34i(PlainObjectBase):
          pass
          
     cdef cppclass Array3Xi(PlainObjectBase):
          pass
          
     cdef cppclass Array42i(PlainObjectBase):
          pass
          
     cdef cppclass Array43i(PlainObjectBase):
          pass
          
     cdef cppclass Array44i(PlainObjectBase):
          pass
          
     cdef cppclass Array4Xi(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX2i(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX3i(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX4i(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXXi(PlainObjectBase):
          pass
          
     cdef cppclass Array2i(PlainObjectBase):
          pass
          
     cdef cppclass Array3i(PlainObjectBase):
          pass
          
     cdef cppclass Array4i(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXi(PlainObjectBase):
          pass
          
     cdef cppclass Array22f(PlainObjectBase):
          pass
          
     cdef cppclass Array23f(PlainObjectBase):
          pass
          
     cdef cppclass Array24f(PlainObjectBase):
          pass
          
     cdef cppclass Array2Xf(PlainObjectBase):
          pass
          
     cdef cppclass Array32f(PlainObjectBase):
          pass
          
     cdef cppclass Array33f(PlainObjectBase):
          pass
          
     cdef cppclass Array34f(PlainObjectBase):
          pass
          
     cdef cppclass Array3Xf(PlainObjectBase):
          pass
          
     cdef cppclass Array42f(PlainObjectBase):
          pass
          
     cdef cppclass Array43f(PlainObjectBase):
          pass
          
     cdef cppclass Array44f(PlainObjectBase):
          pass
          
     cdef cppclass Array4Xf(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX2f(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX3f(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX4f(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXXf(PlainObjectBase):
          pass
          
     cdef cppclass Array2f(PlainObjectBase):
          pass
          
     cdef cppclass Array3f(PlainObjectBase):
          pass
          
     cdef cppclass Array4f(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXf(PlainObjectBase):
          pass
          
     cdef cppclass Array22d(PlainObjectBase):
          pass
          
     cdef cppclass Array23d(PlainObjectBase):
          pass
          
     cdef cppclass Array24d(PlainObjectBase):
          pass
          
     cdef cppclass Array2Xd(PlainObjectBase):
          pass
          
     cdef cppclass Array32d(PlainObjectBase):
          pass
          
     cdef cppclass Array33d(PlainObjectBase):
          pass
          
     cdef cppclass Array34d(PlainObjectBase):
          pass
          
     cdef cppclass Array3Xd(PlainObjectBase):
          pass
          
     cdef cppclass Array42d(PlainObjectBase):
          pass
          
     cdef cppclass Array43d(PlainObjectBase):
          pass
          
     cdef cppclass Array44d(PlainObjectBase):
          pass
          
     cdef cppclass Array4Xd(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX2d(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX3d(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX4d(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXXd(PlainObjectBase):
          pass
          
     cdef cppclass Array2d(PlainObjectBase):
          pass
          
     cdef cppclass Array3d(PlainObjectBase):
          pass
          
     cdef cppclass Array4d(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXd(PlainObjectBase):
          pass
          
     cdef cppclass Array22cf(PlainObjectBase):
          pass
          
     cdef cppclass Array23cf(PlainObjectBase):
          pass
          
     cdef cppclass Array24cf(PlainObjectBase):
          pass
          
     cdef cppclass Array2Xcf(PlainObjectBase):
          pass
          
     cdef cppclass Array32cf(PlainObjectBase):
          pass
          
     cdef cppclass Array33cf(PlainObjectBase):
          pass
          
     cdef cppclass Array34cf(PlainObjectBase):
          pass
          
     cdef cppclass Array3Xcf(PlainObjectBase):
          pass
          
     cdef cppclass Array42cf(PlainObjectBase):
          pass
          
     cdef cppclass Array43cf(PlainObjectBase):
          pass
          
     cdef cppclass Array44cf(PlainObjectBase):
          pass
          
     cdef cppclass Array4Xcf(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX2cf(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX3cf(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX4cf(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXXcf(PlainObjectBase):
          pass
          
     cdef cppclass Array2cf(PlainObjectBase):
          pass
          
     cdef cppclass Array3cf(PlainObjectBase):
          pass
          
     cdef cppclass Array4cf(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXcf(PlainObjectBase):
          pass
          
     cdef cppclass Array22cd(PlainObjectBase):
          pass
          
     cdef cppclass Array23cd(PlainObjectBase):
          pass
          
     cdef cppclass Array24cd(PlainObjectBase):
          pass
          
     cdef cppclass Array2Xcd(PlainObjectBase):
          pass
          
     cdef cppclass Array32cd(PlainObjectBase):
          pass
          
     cdef cppclass Array33cd(PlainObjectBase):
          pass
          
     cdef cppclass Array34cd(PlainObjectBase):
          pass
          
     cdef cppclass Array3Xcd(PlainObjectBase):
          pass
          
     cdef cppclass Array42cd(PlainObjectBase):
          pass
          
     cdef cppclass Array43cd(PlainObjectBase):
          pass
          
     cdef cppclass Array44cd(PlainObjectBase):
          pass
          
     cdef cppclass Array4Xcd(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX2cd(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX3cd(PlainObjectBase):
          pass
          
     cdef cppclass ArrayX4cd(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXXcd(PlainObjectBase):
          pass
          
     cdef cppclass Array2cd(PlainObjectBase):
          pass
          
     cdef cppclass Array3cd(PlainObjectBase):
          pass
          
     cdef cppclass Array4cd(PlainObjectBase):
          pass
          
     cdef cppclass ArrayXcd(PlainObjectBase):
          pass
          


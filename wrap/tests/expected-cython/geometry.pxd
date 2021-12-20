
from gtsam_eigency.core cimport *
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.pair cimport pair
from libcpp.set cimport set
from libcpp.map cimport map
from libcpp cimport bool

cdef extern from "boost/shared_ptr.hpp" namespace "boost":
    cppclass shared_ptr[T]:
        shared_ptr()
        shared_ptr(T*)
        T* get()
        long use_count() const
        T& operator*()

    cdef shared_ptr[T] dynamic_pointer_cast[T,U](const shared_ptr[U]& r)

cdef extern from "gtsam/base/make_shared.h" namespace "gtsam":
    cdef shared_ptr[T] make_shared[T](const T& r)

cdef extern from "gtsam/geometry/Point2.h" namespace "gtsam":
    cdef cppclass CPoint2 "gtsam::Point2":
        CPoint2() except +
        CPoint2(double x, double y) except +

        void argChar(char a) except +
        void argUChar(unsigned char a) except +
        int dim() except +
        void eigenArguments(const VectorXd& v, const MatrixXd& m) except +
        char returnChar() except +
        CVectorNotEigen vectorConfusion() except +
        double x() except +
        double y() except +

cdef class Point2:
    cdef shared_ptr[CPoint2] CPoint2_
    @staticmethod
    cdef Point2 cyCreateFromShared(const shared_ptr[CPoint2]& other)


cdef extern from "gtsam/geometry/Point3.h" namespace "gtsam":
    cdef cppclass CPoint3 "gtsam::Point3":
        CPoint3(double x, double y, double z) except +

        @staticmethod
        CPoint3 StaticFunctionRet "StaticFunctionRet"(double z) except +
        @staticmethod
        double staticFunction "staticFunction"() except +

        double norm() except +

cdef class Point3:
    cdef shared_ptr[CPoint3] CPoint3_
    @staticmethod
    cdef Point3 cyCreateFromShared(const shared_ptr[CPoint3]& other)



cdef extern from "folder/path/to/Test.h":
    cdef cppclass CTest "Test":
        CTest() except +
        CTest(double a, const MatrixXd& b) except +

        void arg_EigenConstRef(const MatrixXd& value) except +
        pair[CTest,shared_ptr[CTest]] create_MixedPtrs() except +
        pair[shared_ptr[CTest],shared_ptr[CTest]] create_ptrs() except +
        void print_ "print"() except +
        shared_ptr[CPoint2] return_Point2Ptr(bool value) except +
        CTest return_Test(shared_ptr[CTest]& value) except +
        shared_ptr[CTest] return_TestPtr(shared_ptr[CTest]& value) except +
        bool return_bool(bool value) except +
        double return_double(double value) except +
        bool return_field(const CTest& t) except +
        int return_int(int value) except +
        MatrixXd return_matrix1(const MatrixXd& value) except +
        MatrixXd return_matrix2(const MatrixXd& value) except +
        pair[VectorXd,MatrixXd] return_pair(const VectorXd& v, const MatrixXd& A) except +
        pair[shared_ptr[CTest],shared_ptr[CTest]] return_ptrs(shared_ptr[CTest]& p1, shared_ptr[CTest]& p2) except +
        size_t return_size_t(size_t value) except +
        string return_string(string value) except +
        VectorXd return_vector1(const VectorXd& value) except +
        VectorXd return_vector2(const VectorXd& value) except +

cdef class Test:
    cdef shared_ptr[CTest] CTest_
    @staticmethod
    cdef Test cyCreateFromShared(const shared_ptr[CTest]& other)


cdef extern from "folder/path/to/Test.h":
    cdef cppclass CMyBase "MyBase":
        pass

cdef class MyBase:
    cdef shared_ptr[CMyBase] CMyBase_
    @staticmethod
    cdef MyBase cyCreateFromShared(const shared_ptr[CMyBase]& other)


cdef extern from "folder/path/to/Test.h":
    cdef cppclass CMyTemplate "MyTemplate"[T](CMyBase):
        CMyTemplate() except +

        void accept_T(const T& value) except +
        void accept_Tptr(shared_ptr[T]& value) except +
        pair[T,shared_ptr[T]] create_MixedPtrs() except +
        pair[shared_ptr[T],shared_ptr[T]] create_ptrs() except +
        T return_T(shared_ptr[T]& value) except +
        shared_ptr[T] return_Tptr(shared_ptr[T]& value) except +
        pair[shared_ptr[T],shared_ptr[T]] return_ptrs(shared_ptr[T]& p1, shared_ptr[T]& p2) except +
        ARG templatedMethod[ARG](const ARG& t) except +

ctypedef CMyTemplate[CPoint2] CMyTemplatePoint2

cdef class MyTemplatePoint2(MyBase):
    cdef shared_ptr[CMyTemplatePoint2] CMyTemplatePoint2_
    @staticmethod
    cdef MyTemplatePoint2 cyCreateFromShared(const shared_ptr[CMyTemplatePoint2]& other)

ctypedef CMyTemplate[MatrixXd] CMyTemplateMatrix

cdef class MyTemplateMatrix(MyBase):
    cdef shared_ptr[CMyTemplateMatrix] CMyTemplateMatrix_
    @staticmethod
    cdef MyTemplateMatrix cyCreateFromShared(const shared_ptr[CMyTemplateMatrix]& other)


cdef extern from "folder/path/to/Test.h":
    cdef cppclass CMyFactor "MyFactor"[POSE,POINT]:
        CMyFactor(size_t key1, size_t key2, double measured, const shared_ptr[CnoiseModel_Base]& noiseModel) except +


ctypedef CMyFactor[CPose2, MatrixXd] CMyFactorPosePoint2

cdef class MyFactorPosePoint2:
    cdef shared_ptr[CMyFactorPosePoint2] CMyFactorPosePoint2_
    @staticmethod
    cdef MyFactorPosePoint2 cyCreateFromShared(const shared_ptr[CMyFactorPosePoint2]& other)


cdef extern from "folder/path/to/Test.h":
    cdef cppclass CMyVector "MyVector"[N]:
        CMyVector() except +



cdef extern from "folder/path/to/Test.h" namespace "":
        VectorXd pxd_aGlobalFunction "aGlobalFunction"() except +
cdef extern from "folder/path/to/Test.h" namespace "":
        VectorXd pxd_overloadedGlobalFunction "overloadedGlobalFunction"(int a) except +
        VectorXd pxd_overloadedGlobalFunction "overloadedGlobalFunction"(int a, double b) except +

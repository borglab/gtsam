# cython: c_string_type=str, c_string_encoding=ascii

cimport numpy as np
import numpy as npp
cimport geometry
from .geometry cimport shared_ptr
from .geometry cimport dynamic_pointer_cast
from .geometry cimport make_shared
# C helper function that copies all arguments into a positional list.
cdef list process_args(list keywords, tuple args, dict kwargs):
   cdef str keyword
   cdef int n = len(args), m = len(keywords)
   cdef list params = list(args)
   assert len(args)+len(kwargs) == m, 'Expected {} arguments'.format(m)
   try:
       return params + [kwargs[keyword] for keyword in keywords[n:]]
   except:
       raise ValueError('Epected arguments ' + str(keywords))
from gtsam_eigency.core cimport *
from libcpp cimport bool

from libcpp.pair cimport pair
from libcpp.string cimport string
from cython.operator cimport dereference as deref


cdef class Point2:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPoint2_ = shared_ptr[CPoint2]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CPoint2_ = shared_ptr[CPoint2](new CPoint2())
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['x', 'y'], args, kwargs)
            x = <double>(__params[0])
            y = <double>(__params[1])
            self.CPoint2_ = shared_ptr[CPoint2](new CPoint2(x, y))
        except (AssertionError, ValueError):
            pass
        if (self.CPoint2_.use_count()==0):
            raise TypeError('Point2 construction failed!')

    @staticmethod
    cdef Point2 cyCreateFromShared(const shared_ptr[CPoint2]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Point2 return_value = Point2(cyCreateFromShared=True)
        return_value.CPoint2_ = other
        return return_value

    def argChar(self, char a):
        self.CPoint2_.get().argChar(a)
    def argUChar(self, unsigned char a):
        self.CPoint2_.get().argUChar(a)
    def dim(self):
        cdef int ret = self.CPoint2_.get().dim()
        return ret
    def eigenArguments(self, np.ndarray v, np.ndarray m):
        v = v.astype(float, order='F', copy=False)
        m = m.astype(float, order='F', copy=False)
        self.CPoint2_.get().eigenArguments(<VectorXd>(Map[VectorXd](v)), <MatrixXd>(Map[MatrixXd](m)))
    def returnChar(self):
        cdef char ret = self.CPoint2_.get().returnChar()
        return ret
    def vectorConfusion(self):
        cdef shared_ptr[CVectorNotEigen] ret = make_shared[CVectorNotEigen](self.CPoint2_.get().vectorConfusion())
        return VectorNotEigen.cyCreateFromShared(ret)
    def x(self):
        cdef double ret = self.CPoint2_.get().x()
        return ret
    def y(self):
        cdef double ret = self.CPoint2_.get().y()
        return ret


cdef class Point3:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CPoint3_ = shared_ptr[CPoint3]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['x', 'y', 'z'], args, kwargs)
            x = <double>(__params[0])
            y = <double>(__params[1])
            z = <double>(__params[2])
            self.CPoint3_ = shared_ptr[CPoint3](new CPoint3(x, y, z))
        except (AssertionError, ValueError):
            pass
        if (self.CPoint3_.use_count()==0):
            raise TypeError('Point3 construction failed!')

    @staticmethod
    cdef Point3 cyCreateFromShared(const shared_ptr[CPoint3]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Point3 return_value = Point3(cyCreateFromShared=True)
        return_value.CPoint3_ = other
        return return_value

    @staticmethod
    def StaticFunctionRet(double z):
        return Point3.cyCreateFromShared(make_shared[CPoint3](CPoint3.StaticFunctionRet(z)))

    @staticmethod
    def staticFunction():
        return CPoint3.staticFunction()


    def norm(self):
        cdef double ret = self.CPoint3_.get().norm()
        return ret


cdef class Test:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CTest_ = shared_ptr[CTest]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CTest_ = shared_ptr[CTest](new CTest())
        except (AssertionError, ValueError):
            pass
        try:
            __params = process_args(['a', 'b'], args, kwargs)
            a = <double>(__params[0])
            b = <np.ndarray>(__params[1])
            assert isinstance(b, np.ndarray) and b.ndim == 2
            b = b.astype(float, order='F', copy=False)
            self.CTest_ = shared_ptr[CTest](new CTest(a, <MatrixXd>(Map[MatrixXd](b))))
        except (AssertionError, ValueError):
            pass
        if (self.CTest_.use_count()==0):
            raise TypeError('Test construction failed!')

    @staticmethod
    cdef Test cyCreateFromShared(const shared_ptr[CTest]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Test return_value = Test(cyCreateFromShared=True)
        return_value.CTest_ = other
        return return_value

    def arg_EigenConstRef(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        self.CTest_.get().arg_EigenConstRef(<MatrixXd>(Map[MatrixXd](value)))
    def create_MixedPtrs(self):
        cdef pair [CTest,shared_ptr[CTest]] ret = self.CTest_.get().create_MixedPtrs()
        return (Test.cyCreateFromShared(make_shared[CTest](ret.first)),Test.cyCreateFromShared(ret.second))
    def create_ptrs(self):
        cdef pair [shared_ptr[CTest],shared_ptr[CTest]] ret = self.CTest_.get().create_ptrs()
        return (Test.cyCreateFromShared(ret.first),Test.cyCreateFromShared(ret.second))
    def __repr__(self):
        strBuf = RedirectCout()
        self.print_('')
        return strBuf.str()
    def print_(self):
        self.CTest_.get().print_()
    def return_Point2Ptr(self, bool value):
        cdef shared_ptr[CPoint2] ret = self.CTest_.get().return_Point2Ptr(value)
        return Point2.cyCreateFromShared(ret)
    def return_Test(self, Test value):
        cdef shared_ptr[CTest] ret = make_shared[CTest](self.CTest_.get().return_Test(value.CTest_))
        return Test.cyCreateFromShared(ret)
    def return_TestPtr(self, Test value):
        cdef shared_ptr[CTest] ret = self.CTest_.get().return_TestPtr(value.CTest_)
        return Test.cyCreateFromShared(ret)
    def return_bool(self, bool value):
        cdef bool ret = self.CTest_.get().return_bool(value)
        return ret
    def return_double(self, double value):
        cdef double ret = self.CTest_.get().return_double(value)
        return ret
    def return_field(self, Test t):
        cdef bool ret = self.CTest_.get().return_field(deref(t.CTest_))
        return ret
    def return_int(self, int value):
        cdef int ret = self.CTest_.get().return_int(value)
        return ret
    def return_matrix1(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CTest_.get().return_matrix1(<MatrixXd>(Map[MatrixXd](value)))
        return ndarray_copy(ret)
    def return_matrix2(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CTest_.get().return_matrix2(<MatrixXd>(Map[MatrixXd](value)))
        return ndarray_copy(ret)
    def return_pair(self, np.ndarray v, np.ndarray A):
        v = v.astype(float, order='F', copy=False)
        A = A.astype(float, order='F', copy=False)
        cdef pair [VectorXd,MatrixXd] ret = self.CTest_.get().return_pair(<VectorXd>(Map[VectorXd](v)), <MatrixXd>(Map[MatrixXd](A)))
        return (ndarray_copy(ret.first).squeeze(),ndarray_copy(ret.second))
    def return_ptrs(self, Test p1, Test p2):
        cdef pair [shared_ptr[CTest],shared_ptr[CTest]] ret = self.CTest_.get().return_ptrs(p1.CTest_, p2.CTest_)
        return (Test.cyCreateFromShared(ret.first),Test.cyCreateFromShared(ret.second))
    def return_size_t(self, size_t value):
        cdef size_t ret = self.CTest_.get().return_size_t(value)
        return ret
    def return_string(self, string value):
        cdef string ret = self.CTest_.get().return_string(value)
        return ret
    def return_vector1(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CTest_.get().return_vector1(<VectorXd>(Map[VectorXd](value)))
        return ndarray_copy(ret).squeeze()
    def return_vector2(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CTest_.get().return_vector2(<VectorXd>(Map[VectorXd](value)))
        return ndarray_copy(ret).squeeze()


cdef class MyBase:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CMyBase_ = shared_ptr[CMyBase]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        if (self.CMyBase_.use_count()==0):
            raise TypeError('MyBase construction failed!')

    @staticmethod
    cdef MyBase cyCreateFromShared(const shared_ptr[CMyBase]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyBase return_value = MyBase(cyCreateFromShared=True)
        return_value.CMyBase_ = other
        return return_value



cdef class MyTemplatePoint2(MyBase):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CMyTemplatePoint2_ = shared_ptr[CMyTemplatePoint2]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CMyTemplatePoint2_ = shared_ptr[CMyTemplatePoint2](new CMyTemplatePoint2())
        except (AssertionError, ValueError):
            pass
        if (self.CMyTemplatePoint2_.use_count()==0):
            raise TypeError('MyTemplatePoint2 construction failed!')
        self.CMyBase_ = <shared_ptr[CMyBase]>(self.CMyTemplatePoint2_)

    @staticmethod
    cdef MyTemplatePoint2 cyCreateFromShared(const shared_ptr[CMyTemplatePoint2]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyTemplatePoint2 return_value = MyTemplatePoint2(cyCreateFromShared=True)
        return_value.CMyTemplatePoint2_ = other
        return_value.CMyBase_ = <shared_ptr[CMyBase]>(other)
        return return_value

    def accept_T(self, Point2 value):
        self.CMyTemplatePoint2_.get().accept_T(deref(value.CPoint2_))
    def accept_Tptr(self, Point2 value):
        self.CMyTemplatePoint2_.get().accept_Tptr(value.CPoint2_)
    def create_MixedPtrs(self):
        cdef pair [CPoint2,shared_ptr[CPoint2]] ret = self.CMyTemplatePoint2_.get().create_MixedPtrs()
        return (Point2.cyCreateFromShared(make_shared[CPoint2](ret.first)),Point2.cyCreateFromShared(ret.second))
    def create_ptrs(self):
        cdef pair [shared_ptr[CPoint2],shared_ptr[CPoint2]] ret = self.CMyTemplatePoint2_.get().create_ptrs()
        return (Point2.cyCreateFromShared(ret.first),Point2.cyCreateFromShared(ret.second))
    def return_T(self, Point2 value):
        cdef shared_ptr[CPoint2] ret = make_shared[CPoint2](self.CMyTemplatePoint2_.get().return_T(value.CPoint2_))
        return Point2.cyCreateFromShared(ret)
    def return_Tptr(self, Point2 value):
        cdef shared_ptr[CPoint2] ret = self.CMyTemplatePoint2_.get().return_Tptr(value.CPoint2_)
        return Point2.cyCreateFromShared(ret)
    def return_ptrs(self, Point2 p1, Point2 p2):
        cdef pair [shared_ptr[CPoint2],shared_ptr[CPoint2]] ret = self.CMyTemplatePoint2_.get().return_ptrs(p1.CPoint2_, p2.CPoint2_)
        return (Point2.cyCreateFromShared(ret.first),Point2.cyCreateFromShared(ret.second))
    def templatedMethodMatrix(self, np.ndarray t):
        t = t.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CMyTemplatePoint2_.get().templatedMethod[MatrixXd](<MatrixXd>(Map[MatrixXd](t)))
        return ndarray_copy(ret)
    def templatedMethodPoint2(self, Point2 t):
        cdef shared_ptr[CPoint2] ret = make_shared[CPoint2](self.CMyTemplatePoint2_.get().templatedMethod[CPoint2](deref(t.CPoint2_)))
        return Point2.cyCreateFromShared(ret)
    def templatedMethodPoint3(self, Point3 t):
        cdef shared_ptr[CPoint3] ret = make_shared[CPoint3](self.CMyTemplatePoint2_.get().templatedMethod[CPoint3](deref(t.CPoint3_)))
        return Point3.cyCreateFromShared(ret)
    def templatedMethodVector(self, np.ndarray t):
        t = t.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CMyTemplatePoint2_.get().templatedMethod[VectorXd](<VectorXd>(Map[VectorXd](t)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_MyTemplatePoint2_MyBase(MyBase parent):
    try:
        return MyTemplatePoint2.cyCreateFromShared(<shared_ptr[CMyTemplatePoint2]>dynamic_pointer_cast[CMyTemplatePoint2,CMyBase](parent.CMyBase_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class MyTemplateMatrix(MyBase):
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CMyTemplateMatrix_ = shared_ptr[CMyTemplateMatrix]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CMyTemplateMatrix_ = shared_ptr[CMyTemplateMatrix](new CMyTemplateMatrix())
        except (AssertionError, ValueError):
            pass
        if (self.CMyTemplateMatrix_.use_count()==0):
            raise TypeError('MyTemplateMatrix construction failed!')
        self.CMyBase_ = <shared_ptr[CMyBase]>(self.CMyTemplateMatrix_)

    @staticmethod
    cdef MyTemplateMatrix cyCreateFromShared(const shared_ptr[CMyTemplateMatrix]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyTemplateMatrix return_value = MyTemplateMatrix(cyCreateFromShared=True)
        return_value.CMyTemplateMatrix_ = other
        return_value.CMyBase_ = <shared_ptr[CMyBase]>(other)
        return return_value

    def accept_T(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        self.CMyTemplateMatrix_.get().accept_T(<MatrixXd>(Map[MatrixXd](value)))
    def accept_Tptr(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        self.CMyTemplateMatrix_.get().accept_Tptr(<MatrixXd>(Map[MatrixXd](value)))
    def create_MixedPtrs(self):
        cdef pair [MatrixXd,shared_ptr[MatrixXd]] ret = self.CMyTemplateMatrix_.get().create_MixedPtrs()
        return (ndarray_copy(ret.first),ndarray_copy(ret.second))
    def create_ptrs(self):
        cdef pair [shared_ptr[MatrixXd],shared_ptr[MatrixXd]] ret = self.CMyTemplateMatrix_.get().create_ptrs()
        return (ndarray_copy(ret.first),ndarray_copy(ret.second))
    def return_T(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CMyTemplateMatrix_.get().return_T(<MatrixXd>(Map[MatrixXd](value)))
        return ndarray_copy(ret)
    def return_Tptr(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef shared_ptr[MatrixXd] ret = self.CMyTemplateMatrix_.get().return_Tptr(<MatrixXd>(Map[MatrixXd](value)))
        return ndarray_copy(ret)
    def return_ptrs(self, np.ndarray p1, np.ndarray p2):
        p1 = p1.astype(float, order='F', copy=False)
        p2 = p2.astype(float, order='F', copy=False)
        cdef pair [shared_ptr[MatrixXd],shared_ptr[MatrixXd]] ret = self.CMyTemplateMatrix_.get().return_ptrs(<MatrixXd>(Map[MatrixXd](p1)), <MatrixXd>(Map[MatrixXd](p2)))
        return (ndarray_copy(ret.first),ndarray_copy(ret.second))
    def templatedMethodMatrix(self, np.ndarray t):
        t = t.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.CMyTemplateMatrix_.get().templatedMethod[MatrixXd](<MatrixXd>(Map[MatrixXd](t)))
        return ndarray_copy(ret)
    def templatedMethodPoint2(self, Point2 t):
        cdef shared_ptr[CPoint2] ret = make_shared[CPoint2](self.CMyTemplateMatrix_.get().templatedMethod[CPoint2](deref(t.CPoint2_)))
        return Point2.cyCreateFromShared(ret)
    def templatedMethodPoint3(self, Point3 t):
        cdef shared_ptr[CPoint3] ret = make_shared[CPoint3](self.CMyTemplateMatrix_.get().templatedMethod[CPoint3](deref(t.CPoint3_)))
        return Point3.cyCreateFromShared(ret)
    def templatedMethodVector(self, np.ndarray t):
        t = t.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.CMyTemplateMatrix_.get().templatedMethod[VectorXd](<VectorXd>(Map[VectorXd](t)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_MyTemplateMatrix_MyBase(MyBase parent):
    try:
        return MyTemplateMatrix.cyCreateFromShared(<shared_ptr[CMyTemplateMatrix]>dynamic_pointer_cast[CMyTemplateMatrix,CMyBase](parent.CMyBase_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class MyVector3:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CMyVector3_ = shared_ptr[CMyVector3]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CMyVector3_ = shared_ptr[CMyVector3](new CMyVector3())
        except (AssertionError, ValueError):
            pass
        if (self.CMyVector3_.use_count()==0):
            raise TypeError('MyVector3 construction failed!')

    @staticmethod
    cdef MyVector3 cyCreateFromShared(const shared_ptr[CMyVector3]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyVector3 return_value = MyVector3(cyCreateFromShared=True)
        return_value.CMyVector3_ = other
        return return_value



cdef class MyVector12:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CMyVector12_ = shared_ptr[CMyVector12]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args([], args, kwargs)
            self.CMyVector12_ = shared_ptr[CMyVector12](new CMyVector12())
        except (AssertionError, ValueError):
            pass
        if (self.CMyVector12_.use_count()==0):
            raise TypeError('MyVector12 construction failed!')

    @staticmethod
    cdef MyVector12 cyCreateFromShared(const shared_ptr[CMyVector12]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyVector12 return_value = MyVector12(cyCreateFromShared=True)
        return_value.CMyVector12_ = other
        return return_value



cdef class MyFactorPosePoint2:
    def __init__(self, *args, **kwargs):
        cdef list __params
        self.CMyFactorPosePoint2_ = shared_ptr[CMyFactorPosePoint2]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        try:
            __params = process_args(['key1', 'key2', 'measured', 'noiseModel'], args, kwargs)
            key1 = <size_t>(__params[0])
            key2 = <size_t>(__params[1])
            measured = <double>(__params[2])
            noiseModel = <noiseModel_Base>(__params[3])
            assert isinstance(noiseModel, noiseModel_Base)
            self.CMyFactorPosePoint2_ = shared_ptr[CMyFactorPosePoint2](new CMyFactorPosePoint2(key1, key2, measured, noiseModel.CnoiseModel_Base_))
        except (AssertionError, ValueError):
            pass
        if (self.CMyFactorPosePoint2_.use_count()==0):
            raise TypeError('MyFactorPosePoint2 construction failed!')

    @staticmethod
    cdef MyFactorPosePoint2 cyCreateFromShared(const shared_ptr[CMyFactorPosePoint2]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyFactorPosePoint2 return_value = MyFactorPosePoint2(cyCreateFromShared=True)
        return_value.CMyFactorPosePoint2_ = other
        return return_value




def aGlobalFunction():
    cdef VectorXd ret = pxd_aGlobalFunction()
    return ndarray_copy(ret).squeeze()
def overloadedGlobalFunction(*args, **kwargs):
    success, results = overloadedGlobalFunction_0(args, kwargs)
    if success:
            return results
    success, results = overloadedGlobalFunction_1(args, kwargs)
    if success:
            return results
    raise TypeError('Could not find the correct overload')
def overloadedGlobalFunction_0(args, kwargs):
    cdef list __params
    cdef VectorXd return_value
    try:
        __params = process_args(['a'], args, kwargs)
        a = <int>(__params[0])
    except:
        return False, None

    return_value = pxd_overloadedGlobalFunction(a)
    return True, ndarray_copy(return_value).squeeze()
def overloadedGlobalFunction_1(args, kwargs):
    cdef list __params
    cdef VectorXd return_value
    try:
        __params = process_args(['a', 'b'], args, kwargs)
        a = <int>(__params[0])
        b = <double>(__params[1])
    except:
        return False, None

    return_value = pxd_overloadedGlobalFunction(a, b)
    return True, ndarray_copy(return_value).squeeze()

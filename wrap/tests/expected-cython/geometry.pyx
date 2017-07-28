cimport numpy as np
import numpy as npp
cimport geometry
from geometry cimport shared_ptr
from geometry cimport dynamic_pointer_cast
from geometry cimport make_shared
from clonedEigency.core cimport *
from libcpp cimport bool

from libcpp.pair cimport pair
from libcpp.string cimport string
from cython.operator cimport dereference as deref


cdef class Point2:
    def __init__(self, *args, **kwargs):
        self.shared_CPoint2_ = shared_ptr[CPoint2]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        elif self.Point2_0(*args, **kwargs):
            pass
        elif self.Point2_1(*args, **kwargs):
            pass
        else:
            raise TypeError('Point2 construction failed!')

    def Point2_0(self, *args, **kwargs):
        if len(args)+len(kwargs) !=0:
            return False
        self.shared_CPoint2_ = shared_ptr[CPoint2](new CPoint2())
        return True

    def Point2_1(self, *args, **kwargs):
        if len(args)+len(kwargs) !=2:
            return False
        __params = kwargs.copy()
        __names = ['x', 'y']
        for i in range(len(args)):
            __params[__names[i]] = args[i]
        try:
            x = <double>(__params['x'])
            y = <double>(__params['y'])
        except:
            return False
        self.shared_CPoint2_ = shared_ptr[CPoint2](new CPoint2(x, y))
        return True


    @staticmethod
    cdef Point2 cyCreateFromShared(const shared_ptr[CPoint2]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Point2 ret = Point2(cyCreateFromShared=True)
        ret.shared_CPoint2_ = other
        return ret
    def argChar(self, char a):
        self.shared_CPoint2_.get().argChar(a)
    def argUChar(self, unsigned char a):
        self.shared_CPoint2_.get().argUChar(a)
    def dim(self):
        cdef int ret = self.shared_CPoint2_.get().dim()
        return ret
    def eigenArguments(self, np.ndarray v, np.ndarray m):
        v = v.astype(float, order='F', copy=False)
        m = m.astype(float, order='F', copy=False)
        self.shared_CPoint2_.get().eigenArguments(<VectorXd>(Map[VectorXd](v)), <MatrixXd>(Map[MatrixXd](m)))
    def returnChar(self):
        cdef char ret = self.shared_CPoint2_.get().returnChar()
        return ret
    def vectorConfusion(self):
        cdef shared_ptr[CVectorNotEigen] ret = make_shared[CVectorNotEigen](self.shared_CPoint2_.get().vectorConfusion())
        return VectorNotEigen.cyCreateFromShared(ret)
    def x(self):
        cdef double ret = self.shared_CPoint2_.get().x()
        return ret
    def y(self):
        cdef double ret = self.shared_CPoint2_.get().y()
        return ret


cdef class Point3:
    def __init__(self, *args, **kwargs):
        self.shared_CPoint3_ = shared_ptr[CPoint3]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        elif self.Point3_0(*args, **kwargs):
            pass
        else:
            raise TypeError('Point3 construction failed!')

    def Point3_0(self, *args, **kwargs):
        if len(args)+len(kwargs) !=3:
            return False
        __params = kwargs.copy()
        __names = ['x', 'y', 'z']
        for i in range(len(args)):
            __params[__names[i]] = args[i]
        try:
            x = <double>(__params['x'])
            y = <double>(__params['y'])
            z = <double>(__params['z'])
        except:
            return False
        self.shared_CPoint3_ = shared_ptr[CPoint3](new CPoint3(x, y, z))
        return True


    @staticmethod
    cdef Point3 cyCreateFromShared(const shared_ptr[CPoint3]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Point3 ret = Point3(cyCreateFromShared=True)
        ret.shared_CPoint3_ = other
        return ret
    @staticmethod
    def StaticFunctionRet(double z):
        return Point3.cyCreateFromShared(make_shared[CPoint3](CPoint3.StaticFunctionRet(z)))

    @staticmethod
    def staticFunction():
        return CPoint3.staticFunction()


    def norm(self):
        cdef double ret = self.shared_CPoint3_.get().norm()
        return ret


cdef class Test:
    def __init__(self, *args, **kwargs):
        self.shared_CTest_ = shared_ptr[CTest]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        elif self.Test_0(*args, **kwargs):
            pass
        elif self.Test_1(*args, **kwargs):
            pass
        else:
            raise TypeError('Test construction failed!')

    def Test_0(self, *args, **kwargs):
        if len(args)+len(kwargs) !=0:
            return False
        self.shared_CTest_ = shared_ptr[CTest](new CTest())
        return True

    def Test_1(self, *args, **kwargs):
        if len(args)+len(kwargs) !=2:
            return False
        __params = kwargs.copy()
        __names = ['a', 'b']
        for i in range(len(args)):
            __params[__names[i]] = args[i]
        if not isinstance(__params[__names[1]], np.ndarray) or not __params[__names[1]].ndim == 2:
            return False
        try:
            a = <double>(__params['a'])
            b = <np.ndarray>(__params['b'])
        except:
            return False
        b = b.astype(float, order='F', copy=False)
        self.shared_CTest_ = shared_ptr[CTest](new CTest(a, <MatrixXd>(Map[MatrixXd](b))))
        return True


    @staticmethod
    cdef Test cyCreateFromShared(const shared_ptr[CTest]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef Test ret = Test(cyCreateFromShared=True)
        ret.shared_CTest_ = other
        return ret
    def arg_EigenConstRef(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        self.shared_CTest_.get().arg_EigenConstRef(<MatrixXd>(Map[MatrixXd](value)))
    def create_MixedPtrs(self):
        cdef pair [CTest,shared_ptr[CTest]] ret = self.shared_CTest_.get().create_MixedPtrs()
        return (Test.cyCreateFromShared(make_shared[CTest](ret.first)),Test.cyCreateFromShared(ret.second))
    def create_ptrs(self):
        cdef pair [shared_ptr[CTest],shared_ptr[CTest]] ret = self.shared_CTest_.get().create_ptrs()
        return (Test.cyCreateFromShared(ret.first),Test.cyCreateFromShared(ret.second))
    def __str__(self):
        self.print_('')
        return ''
    def print_(self):
        self.shared_CTest_.get().print_()
    def return_Point2Ptr(self, bool value):
        cdef shared_ptr[CPoint2] ret = self.shared_CTest_.get().return_Point2Ptr(value)
        return Point2.cyCreateFromShared(ret)
    def return_Test(self, Test value):
        cdef shared_ptr[CTest] ret = make_shared[CTest](self.shared_CTest_.get().return_Test(value.shared_CTest_))
        return Test.cyCreateFromShared(ret)
    def return_TestPtr(self, Test value):
        cdef shared_ptr[CTest] ret = self.shared_CTest_.get().return_TestPtr(value.shared_CTest_)
        return Test.cyCreateFromShared(ret)
    def return_bool(self, bool value):
        cdef bool ret = self.shared_CTest_.get().return_bool(value)
        return ret
    def return_double(self, double value):
        cdef double ret = self.shared_CTest_.get().return_double(value)
        return ret
    def return_field(self, Test t):
        cdef bool ret = self.shared_CTest_.get().return_field(deref(t.shared_CTest_))
        return ret
    def return_int(self, int value):
        cdef int ret = self.shared_CTest_.get().return_int(value)
        return ret
    def return_matrix1(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.shared_CTest_.get().return_matrix1(<MatrixXd>(Map[MatrixXd](value)))
        return ndarray_copy(ret)
    def return_matrix2(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.shared_CTest_.get().return_matrix2(<MatrixXd>(Map[MatrixXd](value)))
        return ndarray_copy(ret)
    def return_pair(self, np.ndarray v, np.ndarray A):
        v = v.astype(float, order='F', copy=False)
        A = A.astype(float, order='F', copy=False)
        cdef pair [VectorXd,MatrixXd] ret = self.shared_CTest_.get().return_pair(<VectorXd>(Map[VectorXd](v)), <MatrixXd>(Map[MatrixXd](A)))
        return (ndarray_copy(ret.first).squeeze(),ndarray_copy(ret.second))
    def return_ptrs(self, Test p1, Test p2):
        cdef pair [shared_ptr[CTest],shared_ptr[CTest]] ret = self.shared_CTest_.get().return_ptrs(p1.shared_CTest_, p2.shared_CTest_)
        return (Test.cyCreateFromShared(ret.first),Test.cyCreateFromShared(ret.second))
    def return_size_t(self, size_t value):
        cdef size_t ret = self.shared_CTest_.get().return_size_t(value)
        return ret
    def return_string(self, string value):
        cdef string ret = self.shared_CTest_.get().return_string(value)
        return ret
    def return_vector1(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.shared_CTest_.get().return_vector1(<VectorXd>(Map[VectorXd](value)))
        return ndarray_copy(ret).squeeze()
    def return_vector2(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.shared_CTest_.get().return_vector2(<VectorXd>(Map[VectorXd](value)))
        return ndarray_copy(ret).squeeze()


cdef class MyBase:
    def __init__(self, *args, **kwargs):
        self.shared_CMyBase_ = shared_ptr[CMyBase]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        else:
            raise TypeError('MyBase construction failed!')

    @staticmethod
    cdef MyBase cyCreateFromShared(const shared_ptr[CMyBase]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyBase ret = MyBase(cyCreateFromShared=True)
        ret.shared_CMyBase_ = other
        return ret


cdef class MyTemplatePoint2(MyBase):
    def __init__(self, *args, **kwargs):
        self.shared_CMyTemplatePoint2_ = shared_ptr[CMyTemplatePoint2]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        elif self.MyTemplatePoint2_0(*args, **kwargs):
            pass
        else:
            raise TypeError('MyTemplatePoint2 construction failed!')
        self.shared_CMyBase_ = <shared_ptr[CMyBase]>(self.shared_CMyTemplatePoint2_)

    def MyTemplatePoint2_0(self, *args, **kwargs):
        if len(args)+len(kwargs) !=0:
            return False
        self.shared_CMyTemplatePoint2_ = shared_ptr[CMyTemplatePoint2](new CMyTemplatePoint2())
        return True


    @staticmethod
    cdef MyTemplatePoint2 cyCreateFromShared(const shared_ptr[CMyTemplatePoint2]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyTemplatePoint2 ret = MyTemplatePoint2(cyCreateFromShared=True)
        ret.shared_CMyTemplatePoint2_ = other
        ret.shared_CMyBase_ = <shared_ptr[CMyBase]>(other)
        return ret
    def accept_T(self, Point2 value):
        self.shared_CMyTemplatePoint2_.get().accept_T(deref(value.shared_CPoint2_))
    def accept_Tptr(self, Point2 value):
        self.shared_CMyTemplatePoint2_.get().accept_Tptr(value.shared_CPoint2_)
    def create_MixedPtrs(self):
        cdef pair [CPoint2,shared_ptr[CPoint2]] ret = self.shared_CMyTemplatePoint2_.get().create_MixedPtrs()
        return (Point2.cyCreateFromShared(make_shared[CPoint2](ret.first)),Point2.cyCreateFromShared(ret.second))
    def create_ptrs(self):
        cdef pair [shared_ptr[CPoint2],shared_ptr[CPoint2]] ret = self.shared_CMyTemplatePoint2_.get().create_ptrs()
        return (Point2.cyCreateFromShared(ret.first),Point2.cyCreateFromShared(ret.second))
    def return_T(self, Point2 value):
        cdef shared_ptr[CPoint2] ret = make_shared[CPoint2](self.shared_CMyTemplatePoint2_.get().return_T(value.shared_CPoint2_))
        return Point2.cyCreateFromShared(ret)
    def return_Tptr(self, Point2 value):
        cdef shared_ptr[CPoint2] ret = self.shared_CMyTemplatePoint2_.get().return_Tptr(value.shared_CPoint2_)
        return Point2.cyCreateFromShared(ret)
    def return_ptrs(self, Point2 p1, Point2 p2):
        cdef pair [shared_ptr[CPoint2],shared_ptr[CPoint2]] ret = self.shared_CMyTemplatePoint2_.get().return_ptrs(p1.shared_CPoint2_, p2.shared_CPoint2_)
        return (Point2.cyCreateFromShared(ret.first),Point2.cyCreateFromShared(ret.second))
    def templatedMethodMatrix(self, np.ndarray t):
        t = t.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.shared_CMyTemplatePoint2_.get().templatedMethod[MatrixXd](<MatrixXd>(Map[MatrixXd](t)))
        return ndarray_copy(ret)
    def templatedMethodPoint2(self, Point2 t):
        cdef shared_ptr[CPoint2] ret = make_shared[CPoint2](self.shared_CMyTemplatePoint2_.get().templatedMethod[CPoint2](deref(t.shared_CPoint2_)))
        return Point2.cyCreateFromShared(ret)
    def templatedMethodPoint3(self, Point3 t):
        cdef shared_ptr[CPoint3] ret = make_shared[CPoint3](self.shared_CMyTemplatePoint2_.get().templatedMethod[CPoint3](deref(t.shared_CPoint3_)))
        return Point3.cyCreateFromShared(ret)
    def templatedMethodVector(self, np.ndarray t):
        t = t.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.shared_CMyTemplatePoint2_.get().templatedMethod[VectorXd](<VectorXd>(Map[VectorXd](t)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_MyTemplatePoint2_MyBase(MyBase parent):
    try:
        return MyTemplatePoint2.cyCreateFromShared(<shared_ptr[CMyTemplatePoint2]>dynamic_pointer_cast[CMyTemplatePoint2,CMyBase](parent.shared_CMyBase_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class MyTemplateMatrix(MyBase):
    def __init__(self, *args, **kwargs):
        self.shared_CMyTemplateMatrix_ = shared_ptr[CMyTemplateMatrix]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        elif self.MyTemplateMatrix_0(*args, **kwargs):
            pass
        else:
            raise TypeError('MyTemplateMatrix construction failed!')
        self.shared_CMyBase_ = <shared_ptr[CMyBase]>(self.shared_CMyTemplateMatrix_)

    def MyTemplateMatrix_0(self, *args, **kwargs):
        if len(args)+len(kwargs) !=0:
            return False
        self.shared_CMyTemplateMatrix_ = shared_ptr[CMyTemplateMatrix](new CMyTemplateMatrix())
        return True


    @staticmethod
    cdef MyTemplateMatrix cyCreateFromShared(const shared_ptr[CMyTemplateMatrix]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyTemplateMatrix ret = MyTemplateMatrix(cyCreateFromShared=True)
        ret.shared_CMyTemplateMatrix_ = other
        ret.shared_CMyBase_ = <shared_ptr[CMyBase]>(other)
        return ret
    def accept_T(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        self.shared_CMyTemplateMatrix_.get().accept_T(<MatrixXd>(Map[MatrixXd](value)))
    def accept_Tptr(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        self.shared_CMyTemplateMatrix_.get().accept_Tptr(<MatrixXd>(Map[MatrixXd](value)))
    def create_MixedPtrs(self):
        cdef pair [MatrixXd,shared_ptr[MatrixXd]] ret = self.shared_CMyTemplateMatrix_.get().create_MixedPtrs()
        return (ndarray_copy(ret.first),ndarray_copy(ret.second))
    def create_ptrs(self):
        cdef pair [shared_ptr[MatrixXd],shared_ptr[MatrixXd]] ret = self.shared_CMyTemplateMatrix_.get().create_ptrs()
        return (ndarray_copy(ret.first),ndarray_copy(ret.second))
    def return_T(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.shared_CMyTemplateMatrix_.get().return_T(<MatrixXd>(Map[MatrixXd](value)))
        return ndarray_copy(ret)
    def return_Tptr(self, np.ndarray value):
        value = value.astype(float, order='F', copy=False)
        cdef shared_ptr[MatrixXd] ret = self.shared_CMyTemplateMatrix_.get().return_Tptr(<MatrixXd>(Map[MatrixXd](value)))
        return ndarray_copy(ret)
    def return_ptrs(self, np.ndarray p1, np.ndarray p2):
        p1 = p1.astype(float, order='F', copy=False)
        p2 = p2.astype(float, order='F', copy=False)
        cdef pair [shared_ptr[MatrixXd],shared_ptr[MatrixXd]] ret = self.shared_CMyTemplateMatrix_.get().return_ptrs(<MatrixXd>(Map[MatrixXd](p1)), <MatrixXd>(Map[MatrixXd](p2)))
        return (ndarray_copy(ret.first),ndarray_copy(ret.second))
    def templatedMethodMatrix(self, np.ndarray t):
        t = t.astype(float, order='F', copy=False)
        cdef MatrixXd ret = self.shared_CMyTemplateMatrix_.get().templatedMethod[MatrixXd](<MatrixXd>(Map[MatrixXd](t)))
        return ndarray_copy(ret)
    def templatedMethodPoint2(self, Point2 t):
        cdef shared_ptr[CPoint2] ret = make_shared[CPoint2](self.shared_CMyTemplateMatrix_.get().templatedMethod[CPoint2](deref(t.shared_CPoint2_)))
        return Point2.cyCreateFromShared(ret)
    def templatedMethodPoint3(self, Point3 t):
        cdef shared_ptr[CPoint3] ret = make_shared[CPoint3](self.shared_CMyTemplateMatrix_.get().templatedMethod[CPoint3](deref(t.shared_CPoint3_)))
        return Point3.cyCreateFromShared(ret)
    def templatedMethodVector(self, np.ndarray t):
        t = t.astype(float, order='F', copy=False)
        cdef VectorXd ret = self.shared_CMyTemplateMatrix_.get().templatedMethod[VectorXd](<VectorXd>(Map[VectorXd](t)))
        return ndarray_copy(ret).squeeze()
def dynamic_cast_MyTemplateMatrix_MyBase(MyBase parent):
    try:
        return MyTemplateMatrix.cyCreateFromShared(<shared_ptr[CMyTemplateMatrix]>dynamic_pointer_cast[CMyTemplateMatrix,CMyBase](parent.shared_CMyBase_))
    except:
        raise TypeError('dynamic cast failed!')


cdef class MyVector3:
    def __init__(self, *args, **kwargs):
        self.shared_CMyVector3_ = shared_ptr[CMyVector3]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        elif self.MyVector3_0(*args, **kwargs):
            pass
        else:
            raise TypeError('MyVector3 construction failed!')

    def MyVector3_0(self, *args, **kwargs):
        if len(args)+len(kwargs) !=0:
            return False
        self.shared_CMyVector3_ = shared_ptr[CMyVector3](new CMyVector3())
        return True


    @staticmethod
    cdef MyVector3 cyCreateFromShared(const shared_ptr[CMyVector3]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyVector3 ret = MyVector3(cyCreateFromShared=True)
        ret.shared_CMyVector3_ = other
        return ret


cdef class MyVector12:
    def __init__(self, *args, **kwargs):
        self.shared_CMyVector12_ = shared_ptr[CMyVector12]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        elif self.MyVector12_0(*args, **kwargs):
            pass
        else:
            raise TypeError('MyVector12 construction failed!')

    def MyVector12_0(self, *args, **kwargs):
        if len(args)+len(kwargs) !=0:
            return False
        self.shared_CMyVector12_ = shared_ptr[CMyVector12](new CMyVector12())
        return True


    @staticmethod
    cdef MyVector12 cyCreateFromShared(const shared_ptr[CMyVector12]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyVector12 ret = MyVector12(cyCreateFromShared=True)
        ret.shared_CMyVector12_ = other
        return ret


cdef class MyFactorPosePoint2:
    def __init__(self, *args, **kwargs):
        self.shared_CMyFactorPosePoint2_ = shared_ptr[CMyFactorPosePoint2]()
        if len(args)==0 and len(kwargs)==1 and kwargs.has_key('cyCreateFromShared'):
            return
        elif self.MyFactorPosePoint2_0(*args, **kwargs):
            pass
        else:
            raise TypeError('MyFactorPosePoint2 construction failed!')

    def MyFactorPosePoint2_0(self, *args, **kwargs):
        if len(args)+len(kwargs) !=4:
            return False
        __params = kwargs.copy()
        __names = ['key1', 'key2', 'measured', 'noiseModel']
        for i in range(len(args)):
            __params[__names[i]] = args[i]
        if not isinstance(__params[__names[3]], noiseModel_Base):
            return False
        try:
            key1 = <size_t>(__params['key1'])
            key2 = <size_t>(__params['key2'])
            measured = <double>(__params['measured'])
            noiseModel = <noiseModel_Base>(__params['noiseModel'])
        except:
            return False
        self.shared_CMyFactorPosePoint2_ = shared_ptr[CMyFactorPosePoint2](new CMyFactorPosePoint2(key1, key2, measured, noiseModel.shared_CnoiseModel_Base_))
        return True


    @staticmethod
    cdef MyFactorPosePoint2 cyCreateFromShared(const shared_ptr[CMyFactorPosePoint2]& other):
        if other.get() == NULL:
            raise RuntimeError('Cannot create object from a nullptr!')
        cdef MyFactorPosePoint2 ret = MyFactorPosePoint2(cyCreateFromShared=True)
        ret.shared_CMyFactorPosePoint2_ = other
        return ret



def aGlobalFunction():
    cdef VectorXd ret = pxd_aGlobalFunction()
    return ndarray_copy(ret).squeeze()
def overloadedGlobalFunction(*args, **kwargs):
    success, results = overloadedGlobalFunction_0(*args, **kwargs)
    if success:
            return results
    success, results = overloadedGlobalFunction_1(*args, **kwargs)
    if success:
            return results
    raise TypeError('Could not find the correct overload')
def overloadedGlobalFunction_0(*args, **kwargs):
    if len(args)+len(kwargs) !=1:
        return False, None
    __params = kwargs.copy()
    __names = ['a']
    for i in range(len(args)):
        __params[__names[i]] = args[i]
    try:
            a = <int>(__params['a'])
    except:
        return False, None
    cdef VectorXd ret = pxd_overloadedGlobalFunction(a)
    return True, ndarray_copy(ret).squeeze()
def overloadedGlobalFunction_1(*args, **kwargs):
    if len(args)+len(kwargs) !=2:
        return False, None
    __params = kwargs.copy()
    __names = ['a', 'b']
    for i in range(len(args)):
        __params[__names[i]] = args[i]
    try:
            a = <int>(__params['a'])
            b = <double>(__params['b'])
    except:
        return False, None
    cdef VectorXd ret = pxd_overloadedGlobalFunction(a, b)
    return True, ndarray_copy(ret).squeeze()

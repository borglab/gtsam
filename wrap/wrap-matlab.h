// header file to be included in MATLAB wrappers
// Copyright (c) 2008 Frank Dellaert, All Rights reserved
// wrapping and unwrapping is done using specialized templates, see
// http://www.cplusplus.com/doc/tutorial/templates.html

extern "C" {
#include <mex.h>
}

#include <list>
#include <string>
#include <sstream>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace boost; // not usual, but for consiseness of generated code

typedef numeric::ublas::vector<double> Vector;
typedef numeric::ublas::matrix<double> Matrix;

//*****************************************************************************
// Utilities
//*****************************************************************************

void error(const char* str) {
  mexErrMsgIdAndTxt("wrap:error", str);
}

mxArray *scalar(mxClassID classid) {
  mwSize dims[1]; dims[0]=1;
  return mxCreateNumericArray(1, dims, classid, mxREAL);
}

mxArray *vector(int m,  mxClassID classid) {
  mwSize dims[1]; dims[0]=m;
  return mxCreateNumericArray(1, dims, classid, mxREAL);
}

mxArray *matrix(int m, int n,  mxClassID classid) {
  mwSize dims[2]; dims[0]=m; dims[1]=n;
  return mxCreateNumericArray(2, dims, mxUINT32_CLASS, mxREAL);
}

//*****************************************************************************
// Check arguments
//*****************************************************************************

void checkArguments(const string& name, int nargout, int nargin, int expected) {
  stringstream err; 
  err << name << " expects " << expected << " arguments, not " << nargin;
  if (nargin!=expected)
    error(err.str().c_str());
}

//*****************************************************************************
// wrapping C++ basis types in MATLAB arrays
//*****************************************************************************

// default wrapping throws an error: only basis types are allowed in wrap
template <typename Class>
mxArray* wrap(Class& value) {
  error("wrap internal error: attempted wrap of invalid type");
}

// specialization to string
// wraps into a character array
template<>
mxArray* wrap<string>(string& value) {
  return mxCreateString(value.c_str());
}

// specialization to bool -> uint32
// Warning: relies on sizeof(UINT32_T)==sizeof(bool)
template<>
mxArray* wrap<bool>(bool& value) {
  mxArray *result = scalar(mxUINT32_CLASS);
  *(bool*)mxGetData(result) = value;
  return result;
}

// specialization to size_t -> uint32
// Warning: relies on sizeof(UINT32_T)==sizeof(size_t)
template<>
mxArray* wrap<size_t>(size_t& value) {
  mxArray *result = scalar(mxUINT32_CLASS);
  *(size_t*)mxGetData(result) = value;
  return result;
}

// specialization to int -> uint32
// Warning: relies on sizeof(INT32_T)==sizeof(int)
template<>
mxArray* wrap<int>(int& value) {
  mxArray *result = scalar(mxINT32_CLASS);
  *(int*)mxGetData(result) = value;
  return result;
}

// specialization to double -> just double
template<>
mxArray* wrap<double>(double& value) {
  return mxCreateDoubleScalar(value);
}

// wrap a const BOOST vector into a double vector
mxArray* wrap_Vector(const Vector& v) {
  int m = v.size();
  mxArray *result = mxCreateDoubleMatrix(m, 1, mxREAL);
  double *data = mxGetPr(result);
  for (int i=0;i<m;i++) data[i]=v(i);
  return result;
}

// specialization to BOOST vector -> double vector
template<>
mxArray* wrap<Vector >(Vector& v) {
  return wrap_Vector(v);
}

// const version
template<>
mxArray* wrap<const Vector >(const Vector& v) {
  return wrap_Vector(v);
}

// wrap a const BOOST MATRIX into a double matrix
mxArray* wrap_Matrix(const Matrix& A) {
  int m = A.size1(), n = A.size2();
  mxArray *result = mxCreateDoubleMatrix(m, n, mxREAL);
  double *data = mxGetPr(result);
  // converts from column-major to row-major
  for (int j=0;j<n;j++) for (int i=0;i<m;i++,data++) *data = A(i,j);
  return result;
}

// specialization to BOOST MATRIX -> double matrix
template<>
mxArray* wrap<Matrix >(Matrix& A) {
  return wrap_Matrix(A);
}

// const version
template<>
mxArray* wrap<const Matrix >(const Matrix& A) {
  return wrap_Matrix(A);
}

//*****************************************************************************
// unwrapping MATLAB arrays into C++ basis types
//*****************************************************************************

// default unwrapping throws an error
// as wrap only supports passing a reference or one of the basic types
template <typename T>
T unwrap(const mxArray* array) {
  error("wrap internal error: attempted unwrap of invalid type");
}

// specialization to string
// expects a character array
// Warning: relies on mxChar==char
template<>
string unwrap<string>(const mxArray* array) {
  char *data = mxArrayToString(array);
  if (data==NULL) error("unwrap<string>: not a character array");
  string str(data);
  mxFree(data);
  return str;
}

// specialization to bool
// returns a pointer to the array data itself
// Warning: relies on sizeof(UINT32_T)==sizeof(bool)
template<>
bool unwrap<bool>(const mxArray* array) {
  return *(bool*)mxGetData(array);
}

// specialization to size_t
// returns a pointer to the array data itself
// Warning: relies on sizeof(UINT32_T)==sizeof(size_t)
template<>
size_t unwrap<size_t>(const mxArray* array) {
  return *(size_t*)mxGetData(array);
}

// specialization to int
// returns a pointer to the array data itself
// Warning: relies on sizeof(INT32_T)==sizeof(int)
template<>
int unwrap<int>(const mxArray* array) {
  return *(int*)mxGetData(array);
}

// specialization to double
// returns a pointer to the array data itself
template<>
double unwrap<double>(const mxArray* array) {
  return *(double*)mxGetData(array);
}

// specialization to BOOST vector
template<>
Vector unwrap< Vector >(const mxArray* array) {
  int m = mxGetM(array), n = mxGetN(array);
  if (mxIsDouble(array)==false || n!=1) error("unwrap<vector>: not a vector");
  double* data = (double*)mxGetData(array);
  Vector v(m);
  copy(data,data+m,v.begin());
  return v;
}

// specialization to BOOST matrix
template<>
Matrix unwrap< Matrix >(const mxArray* array) {
  if (mxIsDouble(array)==false) error("unwrap<matrix>: not a matrix");
  int m = mxGetM(array), n = mxGetN(array);
  double* data = (double*)mxGetData(array);
  Matrix A(m,n);
  // converts from row-major to column-major
  for (int j=0;j<n;j++) for (int i=0;i<m;i++,data++) A(i,j) = *data;
  return A;
}

//*****************************************************************************
// Shared Pointer Handle
// inspired by mexhandle, but using shared_ptr
//*****************************************************************************

template<typename T> class Collector;

template <typename T>
class ObjectHandle {
private:
  ObjectHandle* signature; // use 'this' as a unique object signature 
  const std::type_info* type; // type checking information
  shared_ptr<T> t; // object pointer

public:
  // Constructor for free-store allocated objects.
  // Creates shared pointer, will delete if is last one to hold pointer
ObjectHandle(T* ptr) : type(&typeid(T)), t(shared_ptr<T>(ptr)) { 
    signature = this; 
    Collector<T>::register_handle(this);
  } 

  // Constructor for shared pointers
  // Creates shared pointer, will delete if is last one to hold pointer
ObjectHandle(shared_ptr<T> ptr) : type(&typeid(T)), t(ptr) { 
    signature= this; 
  } 

  ~ObjectHandle() { 
    // object is in shared_ptr, will be automatically deleted
    signature= 0; // destroy signature
  } 

  // Get the actual object contained by handle
  shared_ptr<T> get_object() const { return t; }

  // Print the mexhandle for debugging
  void print(const char* str) {
    mexPrintf("mexhandle %s:\n", str);
    mexPrintf("  signature = %d:\n", signature);
    mexPrintf("  pointer   = %d:\n", t.get());
  }

  // Convert ObjectHandle<T> to a mxArray handle (to pass back from mex-function).
  // Create a numeric array as handle for an ObjectHandle.
  // We ASSUME we can store object pointer in the mxUINT32 element of mxArray.
  mxArray* to_mex_handle() 
  {
    mxArray* handle  = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *reinterpret_cast<ObjectHandle<T>**>(mxGetPr(handle)) = this;
    return handle;
  }

  string type_name() const {return type->name();} 

  // Convert mxArray (passed to mex-function) to an ObjectHandle<T>.
  // Import a handle from MatLab as a mxArray of UINT32. Check that
  // it is actually a pointer to an ObjectHandle<T>.
  static ObjectHandle* from_mex_handle(const mxArray* handle) 
  {
    if (mxGetClassID(handle) != mxUINT32_CLASS 
	|| mxIsComplex(handle) || mxGetM(handle)!=1 || mxGetN(handle)!=1)
      error("Parameter is not an ObjectHandle type.");

    // We *assume* we can store ObjectHandle<T> pointer in the mxUINT32 of handle
    ObjectHandle* obj = *reinterpret_cast<ObjectHandle**>(mxGetPr(handle));

    if (!obj) // gross check to see we don't have an invalid pointer
      error("Parameter is NULL. It does not represent an ObjectHandle object.");
    // TODO: change this for max-min check for pointer values

    if (obj->signature != obj) // check memory has correct signature
      error("Parameter does not represent an ObjectHandle object.");

/*
  if (*(obj->type) != typeid(T)) { // check type 
  mexPrintf("Given: <%s>, Required: <%s>.\n", obj->type_name(), typeid(T).name());
  error("Given ObjectHandle does not represent the correct type.");
  }
*/

    return obj;
  }

  friend class Collector<T>; // allow Collector access to signature
};

// --------------------------------------------------------- 
// ------------------ Garbage Collection -------------------
// --------------------------------------------------------- 

// Garbage collection singleton (one collector object for each type T).
// Ensures that registered handles are deleted when the dll is released (they
// may also be deleted previously without problem).
//    The Collector provides protection against resource leaks in the case
// where 'clear all' is called in MatLab. (This is because MatLab will call
// the destructors of statically allocated objects but not free-store allocated
// objects.)
template <typename T>
class Collector {
  typedef ObjectHandle<T> Handle;
  typedef std::list< Handle* > ObjList;
  typedef typename ObjList::iterator iterator;
  ObjList objlist;
public:
  ~Collector() {
    for (iterator i= objlist.begin(); i!=objlist.end(); ++i) {
      if ((*i)->signature == *i) // check for valid signature
	delete *i;
    }
  }

  static void register_handle (Handle* obj) {
    static Collector singleton;
    singleton.objlist.push_back(obj);
  }

private: // prevent construction
  Collector() {}
  Collector(const Collector&);
};

//*****************************************************************************
// wrapping C++ objects in a MATLAB proxy class
//*****************************************************************************

/* 
 For every C++ class Class, a matlab proxy class @Class/Class.m object
 is created. Its constructor will check which of the C++ constructors
 needs to be called, based on nr of arguments. It then calls the
 corresponding mex function new_Class_signature, which will create a
 C++ object using new, and pass the pointer to wrap_constructed
 (below). This creates a mexhandle and returns it to the proxy class
 constructor, which assigns it to self. Matlab owns this handle now.
*/
template <typename Class>
mxArray* wrap_constructed(Class* pointer, const char *classname) {
  ObjectHandle<Class>* handle = new ObjectHandle<Class>(pointer);
  return handle->to_mex_handle();
}

/*
 [create_object] creates a MATLAB proxy class object with a mexhandle
 in the self property. Matlab does not allow the creation of matlab
 objects from within mex files, hence we resort to an ugly trick: we
 invoke the proxy class constructor by calling MATLAB, and pass 13
 dummy arguments to let the constructor know we want an object without
 the self property initialized. We then assign the mexhandle to self.
*/
// TODO: think about memory
mxArray* create_object(const char *classname, mxArray* h) {
  mxArray *result;
  mxArray* dummy[13] = {h,h,h,h,h, h,h,h,h,h, h,h,h};
  mexCallMATLAB(1,&result,13,dummy,classname);
  mxSetProperty(result, 0, "self", h);
  return result;
}

/*
 When the user calls a method that returns a shared pointer, we create
 an ObjectHandle from the shared_pointer and return it as a proxy
 class to matlab.
*/
template <typename Class>
mxArray* wrap_shared_ptr(shared_ptr< Class > shared_ptr, const char *classname) {
  ObjectHandle<Class>* handle = new ObjectHandle<Class>(shared_ptr);
  return create_object(classname,handle->to_mex_handle());
}

//*****************************************************************************
// unwrapping a MATLAB proxy class to a C++ object reference
//*****************************************************************************

/*
 Besides the basis types, the only other argument type allowed is a
 shared pointer to a C++ object. In this case, matlab needs to pass a
 proxy class object to the mex function. [unwrap_shared_ptr] extracts
 the ObjectHandle from the self property, and returns a shared pointer
 to the object.
*/
template <typename Class>
shared_ptr<Class> unwrap_shared_ptr(const mxArray* obj, const string& className) {
  bool isClass = mxIsClass(obj, className.c_str());
  if (!isClass) {
    mexPrintf("Expected %s, got %s\n", className.c_str(), mxGetClassName(obj));
    error("Argument has wrong type.");
  }
  mxArray* mxh = mxGetProperty(obj,0,"self");
  if (mxh==NULL) error("unwrap_reference: invalid wrap object");
  ObjectHandle<Class>* handle = ObjectHandle<Class>::from_mex_handle(mxh);
  return handle->get_object();
}

//*****************************************************************************

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file matlab.h
 * @brief header file to be included in MATLAB wrappers
 * @date 2008
 * @author Frank Dellaert
 * @author Alex Cunningham
 * @author Andrew Melim
 * @author Richard Roberts
 *
 * wrapping and unwrapping is done using specialized templates, see
 * http://www.cplusplus.com/doc/tutorial/templates.html
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/utilities.h>

using gtsam::Vector;
using gtsam::Matrix;
using gtsam::Point2;
using gtsam::Point3;

extern "C" {
#include <mex.h>
}

#include <list>
#include <set>
#include <sstream>
#include <streambuf>
#include <string>
#include <typeinfo>

using namespace std;

// start GTSAM Specifics /////////////////////////////////////////////////
// to enable Matrix and Vector constructor for SharedGaussian:
#define GTSAM_MAGIC_GAUSSIAN
// end GTSAM Specifics /////////////////////////////////////////////////

#if defined(__LP64__) || defined(_WIN64)
// 64-bit
#define mxUINT32OR64_CLASS mxUINT64_CLASS
#else
#define mxUINT32OR64_CLASS mxUINT32_CLASS
#endif

// "Unique" key to signal calling the matlab object constructor with a raw pointer
// to a shared pointer of the same C++ object type as the MATLAB type.
// Also present in utilities.h
static const std::uint64_t ptr_constructor_key =
  (std::uint64_t('G') << 56) |
  (std::uint64_t('T') << 48) |
  (std::uint64_t('S') << 40) |
  (std::uint64_t('A') << 32) |
  (std::uint64_t('M') << 24) |
  (std::uint64_t('p') << 16) |
  (std::uint64_t('t') << 8) |
  (std::uint64_t('r'));

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

void checkScalar(const mxArray* array, const char* str) {
  int m = mxGetM(array), n = mxGetN(array);
  if (m!=1 || n!=1)
    mexErrMsgIdAndTxt("wrap: not a scalar in ", str);
}

// Replacement streambuf for cout that writes to the MATLAB console
// Thanks to http://stackoverflow.com/a/249008
class mstream : public std::streambuf {
protected:
  virtual std::streamsize xsputn(const char *s, std::streamsize n) {
    mexPrintf("%.*s",n,s);
    return n;
  }
  virtual int overflow(int c = EOF) {
    if (c != EOF) {
      mexPrintf("%.1s",&c);
    }
    return 1;
  }
};

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
mxArray* wrap(const Class& value) {
  error("wrap internal error: attempted wrap of invalid type");
  return 0;
}

// specialization to string
// wraps into a character array
template<>
mxArray* wrap<string>(const string& value) {
  return mxCreateString(value.c_str());
}

// specialization to char
template<>
mxArray* wrap<char>(const char& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(char*)mxGetData(result) = value;
  return result;
}

// specialization to unsigned char
template<>
mxArray* wrap<unsigned char>(const unsigned char& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(unsigned char*)mxGetData(result) = value;
  return result;
}

// specialization to bool
template<>
mxArray* wrap<bool>(const bool& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(bool*)mxGetData(result) = value;
  return result;
}

// specialization to size_t
template<>
mxArray* wrap<size_t>(const size_t& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(size_t*)mxGetData(result) = value;
  return result;
}

// specialization to int
template<>
mxArray* wrap<int>(const int& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(int*)mxGetData(result) = value;
  return result;
}

// specialization to double -> just double
template<>
mxArray* wrap<double>(const double& value) {
  return mxCreateDoubleScalar(value);
}

// wrap a const Eigen vector into a double vector
mxArray* wrap_Vector(const gtsam::Vector& v) {
  int m = v.size();
  mxArray *result = mxCreateDoubleMatrix(m, 1, mxREAL);
  double *data = mxGetPr(result);
  for (int i=0;i<m;i++) data[i]=v(i);
  return result;
}

// specialization to Eigen vector -> double vector
template<>
mxArray* wrap<gtsam::Vector >(const gtsam::Vector& v) {
  return wrap_Vector(v);
}

// specialization to Eigen vector -> double vector
template<>
mxArray* wrap<gtsam::Point2 >(const gtsam::Point2& v) {
  return wrap_Vector(v);
}

// specialization to Eigen vector -> double vector
template<>
mxArray* wrap<gtsam::Point3 >(const gtsam::Point3& v) {
  return wrap_Vector(v);
}

// wrap a const Eigen MATRIX into a double matrix
mxArray* wrap_Matrix(const gtsam::Matrix& A) {
  int m = A.rows(), n = A.cols();
#ifdef DEBUG_WRAP
  mexPrintf("wrap_Matrix called with A = \n", m,n);
  gtsam::print(A);
#endif
  mxArray *result = mxCreateDoubleMatrix(m, n, mxREAL);
  double *data = mxGetPr(result);
  // converts from column-major to row-major
  for (int j=0;j<n;j++) for (int i=0;i<m;i++,data++) *data = A(i,j);
  return result;
}

// specialization to Eigen MATRIX -> double matrix
template<>
mxArray* wrap<gtsam::Matrix >(const gtsam::Matrix& A) {
  return wrap_Matrix(A);
}

template <typename T>
mxArray* wrap_enum(const T x, const std::string& classname) {
  // create double array to store value in
  mxArray* a = mxCreateDoubleMatrix(1, 1, mxREAL);
  double* data = mxGetPr(a);
  data[0] = static_cast<double>(x);

  // convert to Matlab enumeration type
  mxArray* result;
  mexCallMATLAB(1, &result, 1, &a, classname.c_str());

  return result;
}

//*****************************************************************************
// unwrapping MATLAB arrays into C++ basic types
//*****************************************************************************

// default unwrapping throws an error
// as wrap only supports passing a reference or one of the basic types
template <typename T>
T unwrap(const mxArray* array) {
  error("wrap internal error: attempted unwrap of invalid type");
  return T();
}

template <typename T>
shared_ptr<T> unwrap_enum(const mxArray* array) {
  // Make duplicate to remove const-ness
  mxArray* a = mxDuplicateArray(array);
  std::cout << "unwrap enum type: " << typeid(array).name() << std::endl;

  // convert void* to int32* array
  mxArray* a_int32;
  mexCallMATLAB(1, &a_int32, 1, &a, "int32");

  // Get the value in the input array
  int32_T* value = (int32_T*)mxGetData(a_int32);
  // cast int32 to enum type
  return std::make_shared<T>(static_cast<T>(*value));
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

// Check for 64-bit, as Mathworks says mxGetScalar only good for 32 bit
template <typename T>
T myGetScalar(const mxArray* array) {
  switch (mxGetClassID(array)) {
    case mxINT64_CLASS:
      return (T) *(std::int64_t*) mxGetData(array);
    case mxUINT64_CLASS:
      return (T) *(std::uint64_t*) mxGetData(array);
    default:
      // hope for the best!
      return (T) mxGetScalar(array);
  }
}

// specialization to bool
template<>
bool unwrap<bool>(const mxArray* array) {
  checkScalar(array,"unwrap<bool>");
  return myGetScalar<bool>(array);
}

// specialization to char
template<>
char unwrap<char>(const mxArray* array) {
  checkScalar(array,"unwrap<char>");
  return myGetScalar<char>(array);
}

// specialization to unsigned char
template<>
unsigned char unwrap<unsigned char>(const mxArray* array) {
  checkScalar(array,"unwrap<unsigned char>");
  return myGetScalar<unsigned char>(array);
}

// specialization to int
template<>
int unwrap<int>(const mxArray* array) {
  checkScalar(array,"unwrap<int>");
  return myGetScalar<int>(array);
}

// specialization to size_t
template<>
size_t unwrap<size_t>(const mxArray* array) {
  checkScalar(array, "unwrap<size_t>");
  return myGetScalar<size_t>(array);
}

// specialization to double
template<>
double unwrap<double>(const mxArray* array) {
  checkScalar(array,"unwrap<double>");
  return myGetScalar<double>(array);
}

// specialization to Eigen vector
template<>
gtsam::Vector unwrap< gtsam::Vector >(const mxArray* array) {
  int m = mxGetM(array), n = mxGetN(array);
  if (mxIsDouble(array)==false || n!=1) error("unwrap<vector>: not a vector");
#ifdef DEBUG_WRAP
  mexPrintf("unwrap< gtsam::Vector > called with %dx%d argument\n", m,n);
#endif
  double* data = (double*)mxGetData(array);
  gtsam::Vector v(m);
  for (int i=0;i<m;i++,data++) v(i) = *data;
#ifdef DEBUG_WRAP
  gtsam::print(v);
#endif
  return v;
}

// specialization to Point2
template<>
gtsam::Point2 unwrap< gtsam::Point2 >(const mxArray* array) {
  int m = mxGetM(array), n = mxGetN(array);
  if (mxIsDouble(array)==false || n!=1) error("unwrap<vector>: not a vector");
#ifdef DEBUG_WRAP
  mexPrintf("unwrap< gtsam::Vector > called with %dx%d argument\n", m,n);
#endif
  double* data = (double*)mxGetData(array);
  gtsam::Vector v(m);
  for (int i=0;i<m;i++,data++) v(i) = *data;
#ifdef DEBUG_WRAP
  gtsam::print(v);
#endif
  return v;
}

// specialization to Point3
template<>
gtsam::Point3 unwrap< gtsam::Point3 >(const mxArray* array) {
  int m = mxGetM(array), n = mxGetN(array);
  if (mxIsDouble(array)==false || n!=1) error("unwrap<vector>: not a vector");
#ifdef DEBUG_WRAP
  mexPrintf("unwrap< gtsam::Vector > called with %dx%d argument\n", m,n);
#endif
  double* data = (double*)mxGetData(array);
  gtsam::Vector v(m);
  for (int i=0;i<m;i++,data++) v(i) = *data;
#ifdef DEBUG_WRAP
  gtsam::print(v);
#endif
  return v;
}


// specialization to Eigen matrix
template<>
gtsam::Matrix unwrap< gtsam::Matrix >(const mxArray* array) {
  if (mxIsDouble(array)==false) error("unwrap<matrix>: not a matrix");
  int m = mxGetM(array), n = mxGetN(array);
#ifdef DEBUG_WRAP
  mexPrintf("unwrap< gtsam::Matrix > called with %dx%d argument\n", m,n);
#endif
  double* data = (double*)mxGetData(array);
  gtsam::Matrix A(m,n);
  // converts from row-major to column-major
  for (int j=0;j<n;j++) for (int i=0;i<m;i++,data++) A(i,j) = *data;
#ifdef DEBUG_WRAP
  gtsam::print(A);
#endif
  return A;
}

/*
 [create_object] creates a MATLAB proxy class object with a mexhandle
 in the self property. Matlab does not allow the creation of matlab
 objects from within mex files, hence we resort to an ugly trick: we
 invoke the proxy class constructor by calling MATLAB with a special
 uint64 value ptr_constructor_key and the pointer itself.  MATLAB
 allocates the object.  Then, the special constructor in our wrap code
 that is activated when the ptr_constructor_key is passed in passes
 the pointer back into a C++ function to add the pointer to its
 collector.  We go through this extra "C++ to MATLAB to C++ step" in
 order to be able to add to the collector could be in a different wrap
 module.
*/
mxArray* create_object(const std::string& classname, void *pointer, bool isVirtual, const char *rttiName) {
  mxArray *result;
  mxArray *input[3];
  int nargin = 2;
  // First input argument is pointer constructor key
  input[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
  *reinterpret_cast<std::uint64_t*>(mxGetData(input[0])) = ptr_constructor_key;
  // Second input argument is the pointer
  input[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<void**>(mxGetData(input[1])) = pointer;
  // If the class is virtual, use the RTTI name to look up the derived matlab type
  const char *derivedClassName;
  if(isVirtual) {
    const mxArray *rttiRegistry = mexGetVariablePtr("global", "gtsamwrap_rttiRegistry");
    if(!rttiRegistry)
      mexErrMsgTxt(
      "gtsam wrap:  RTTI registry is missing - it could have been cleared from the workspace."
      "  You can issue 'clear all' to completely clear the workspace, and next time a wrapped object is"
      " created the RTTI registry will be recreated.");
    const mxArray *derivedNameMx = mxGetField(rttiRegistry, 0, rttiName);
    if(!derivedNameMx)
      mexErrMsgTxt((
      "gtsam wrap:  The derived class type " + string(rttiName) + " was not found in the RTTI registry.  "
      "Try calling 'clear all' twice consecutively - we have seen things not get unloaded properly the "
      "first time.  If this does not work, this may indicate an inconsistency in your wrap interface file.  "
      "The most likely cause for this is that a base class was marked virtual in the wrap interface "
      "definition header file for gtsam or for your module, but a derived type was returned by a C++ "
      "function and that derived type was not marked virtual (or was not specified in the wrap interface "
      "definition header at all).").c_str());
    size_t strLen = mxGetN(derivedNameMx);
    char *buf = new char[strLen+1];
    if(mxGetString(derivedNameMx, buf, strLen+1))
      mexErrMsgTxt("gtsam wrap:  Internal error reading RTTI table, try 'clear all' to clear your workspace and reinitialize the toolbox.");
    derivedClassName = buf;
    input[2] = mxCreateString("void");
    nargin = 3;
  } else {
    derivedClassName = classname.c_str();
  }
  // Call special pointer constructor, which sets 'self'
  mexCallMATLAB(1,&result, nargin, input, derivedClassName);
  // Deallocate our memory
  mxDestroyArray(input[0]);
  mxDestroyArray(input[1]);
  if(isVirtual) {
    mxDestroyArray(input[2]);
    delete[] derivedClassName;
  }
  return result;
}

/*
 When the user calls a method that returns a shared pointer, we create
 an ObjectHandle from the shared_pointer and return it as a proxy
 class to matlab.
*/
template <typename Class>
mxArray* wrap_shared_ptr(std::shared_ptr< Class > shared_ptr, const std::string& matlabName, bool isVirtual) {
  // Create actual class object from out pointer
  mxArray* result;
  if(isVirtual) {
    std::shared_ptr<void> void_ptr(shared_ptr);
    result = create_object(matlabName, &void_ptr, isVirtual, typeid(*shared_ptr).name());
  } else {
    std::shared_ptr<Class> *heapPtr = new std::shared_ptr<Class>(shared_ptr);
    result = create_object(matlabName, heapPtr, isVirtual, "");
  }
  return result;
}

template <typename Class>
std::shared_ptr<Class> unwrap_shared_ptr(const mxArray* obj, const string& propertyName) {

  mxArray* mxh = mxGetProperty(obj,0, propertyName.c_str());
  if (mxGetClassID(mxh) != mxUINT32OR64_CLASS || mxIsComplex(mxh)
    || mxGetM(mxh) != 1 || mxGetN(mxh) != 1) error(
    "Parameter is not an Shared type.");

  std::shared_ptr<Class>* spp = *reinterpret_cast<std::shared_ptr<Class>**> (mxGetData(mxh));
  return *spp;
}

template <typename Class>
Class* unwrap_ptr(const mxArray* obj, const string& propertyName) {

  mxArray* mxh = mxGetProperty(obj,0, propertyName.c_str());
  Class* x = reinterpret_cast<Class*> (mxGetData(mxh));
  return x;
}

//// throw an error if unwrap_shared_ptr is attempted for an Eigen Vector
//template <>
//Vector unwrap_shared_ptr<Vector>(const mxArray* obj, const string& propertyName) {
//  bool unwrap_shared_ptr_Vector_attempted = false;
//  static_assert(unwrap_shared_ptr_Vector_attempted, "Vector cannot be unwrapped as a shared pointer");
//  return Vector();
//}

//// throw an error if unwrap_shared_ptr is attempted for an Eigen Matrix
//template <>
//Matrix unwrap_shared_ptr<Matrix>(const mxArray* obj, const string& propertyName) {
//  bool unwrap_shared_ptr_Matrix_attempted = false;
//  static_assert(unwrap_shared_ptr_Matrix_attempted, "Matrix cannot be unwrapped as a shared pointer");
//  return Matrix();
//}


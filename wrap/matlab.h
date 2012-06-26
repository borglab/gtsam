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
 *
 * wrapping and unwrapping is done using specialized templates, see
 * http://www.cplusplus.com/doc/tutorial/templates.html
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>

using gtsam::Vector;
using gtsam::Matrix;
using gtsam::noiseModel::Base;
using gtsam::noiseModel::Gaussian;
using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using gtsam::noiseModel::Unit;

extern "C" {
#include <mex.h>
}

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <list>
#include <string>
#include <sstream>
#include <typeinfo>
#include <set>

using namespace std;
using namespace boost; // not usual, but for conciseness of generated code

// start GTSAM Specifics /////////////////////////////////////////////////
// to enable Matrix and Vector constructor for SharedGaussian:
#define GTSAM_MAGIC_GAUSSIAN
// end GTSAM Specifics /////////////////////////////////////////////////

#ifdef __LP64__
// 64-bit Mac
#define mxUINT32OR64_CLASS mxUINT64_CLASS
#else
#define mxUINT32OR64_CLASS mxUINT32_CLASS
#endif

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

// specialization to char
template<>
mxArray* wrap<char>(char& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(char*)mxGetData(result) = value;
  return result;
}

// specialization to unsigned char
template<>
mxArray* wrap<unsigned char>(unsigned char& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(unsigned char*)mxGetData(result) = value;
  return result;
}

// specialization to bool
template<>
mxArray* wrap<bool>(bool& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(bool*)mxGetData(result) = value;
  return result;
}

// specialization to size_t
template<>
mxArray* wrap<size_t>(size_t& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(size_t*)mxGetData(result) = value;
  return result;
}

// specialization to int
template<>
mxArray* wrap<int>(int& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(int*)mxGetData(result) = value;
  return result;
}

// specialization to double -> just double
template<>
mxArray* wrap<double>(double& value) {
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
mxArray* wrap<gtsam::Vector >(gtsam::Vector& v) {
  return wrap_Vector(v);
}

// const version
template<>
mxArray* wrap<const gtsam::Vector >(const gtsam::Vector& v) {
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
mxArray* wrap<gtsam::Matrix >(gtsam::Matrix& A) {
  return wrap_Matrix(A);
}

// const version
template<>
mxArray* wrap<const gtsam::Matrix >(const gtsam::Matrix& A) {
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

// Check for 64-bit, as Mathworks says mxGetScalar only good for 32 bit
template <typename T>
T myGetScalar(const mxArray* array) {
	switch (mxGetClassID(array)) {
		case mxINT64_CLASS:
			return (T) *(int64_t*) mxGetData(array);
		case mxUINT64_CLASS:
			return (T) *(uint64_t*) mxGetData(array);
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
 * Similar to create object, this also collects the shared_ptr in addition
 * to creating the dummy object. Mainly used for static constructor methods
 * which don't have direct access to the function.
 */
mxArray* create_collect_object(const char *classname, mxArray* h){
  mxArray *result;
  //First arg is a flag to collect
  mxArray* dummy[14] = {h,h,h,h,h, h,h,h,h,h, h,h,h,h};
  mexCallMATLAB(1,&result,14,dummy,classname);
  mxSetProperty(result, 0, "self", h);
  cout << "Return collect" << endl;
  return result;
}

/*
 When the user calls a method that returns a shared pointer, we create
 an ObjectHandle from the shared_pointer and return it as a proxy
 class to matlab.
*/
template <typename Class>
mxArray* wrap_shared_ptr(boost::shared_ptr< Class >* shared_ptr, const char *classname) {
  mxArray* mxh = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<boost::shared_ptr<Class>**> (mxGetPr(mxh)) = shared_ptr;
  cout << "wrapped:" << mxh << endl << "end wrap" << endl;
  //return mxh;
  return create_object(classname, mxh);
}

template <typename Class>
mxArray* wrap_collect_shared_ptr(boost::shared_ptr< Class >* shared_ptr, const char *classname) {
  mxArray* mxh = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<boost::shared_ptr<Class>**> (mxGetPr(mxh)) = shared_ptr;
  cout << "wrapped:" << mxh << endl << "end wrap" << endl;
  //return mxh;
  return create_collect_object(classname, mxh);
}

template <typename Class>
boost::shared_ptr<Class> unwrap_shared_ptr(const mxArray* obj, const string& className) {
    cout << "UNWRAP CALL" << endl;

  mxArray* mxh = mxGetProperty(obj,0,"self");
  if (mxGetClassID(mxh) != mxUINT32OR64_CLASS || mxIsComplex(mxh)
    || mxGetM(mxh) != 1 || mxGetN(mxh) != 1) error(
    "Parameter is not an Shared type.");

  cout << "unwrapped:" << mxh << endl;
  boost::shared_ptr<Class>* spp = *reinterpret_cast<boost::shared_ptr<Class>**> (mxGetPr(mxh));
  cout << "unwrapped:" << spp << endl;
  return *spp;
}


//*****************************************************************************
// Shared Pointer Handle
// inspired by mexhandle, but using shared_ptr
//*****************************************************************************

template<typename T>
class ObjectHandle {
private:
	ObjectHandle* signature; // use 'this' as a unique object signature
	const std::type_info* type; // type checking information
	boost::shared_ptr<T> t; // object pointer

public:
	// Constructor for free-store allocated objects.
	// Creates shared pointer, will delete if is last one to hold pointer
	ObjectHandle(T* ptr) :
		type(&typeid(T)), t(ptr) {
		signature = this;
		mexPrintf("Created Shared Pointer use_count = %li\n", t.use_count());
		mexPrintf("Created Pointer points to %d\n", t.get());
		
	}

	// Constructor for shared pointers
	// Creates shared pointer, will delete if is last one to hold pointer
	ObjectHandle(boost::shared_ptr<T> shared_ptr) :
		/*type(&typeid(T)),*/ t(shared_ptr) {
		signature = this;
		mexPrintf("Created sp from sp use_count = %li\n", t.use_count());
		mexPrintf("Created sp from sp points to %d\n", t.get());
	}

	~ObjectHandle() {
		// object is in shared_ptr, will be automatically deleted
		signature = 0; // destroy signature
    // std::cout << "ObjectHandle destructor" << std::endl;
	}

	// Get the actual object contained by handle
	boost::shared_ptr<T> get_object() const {
		return t;
	}

	// Print the mexhandle for debugging
	void print(const char* str) {
		mexPrintf("mexhandle %s:\n", str);
		mexPrintf("  signature = %d:\n", signature);
		mexPrintf("  pointer   = %d:\n", t.get());
	}

	// Convert ObjectHandle<T> to a mxArray handle (to pass back from mex-function).
	// Create a numeric array as handle for an ObjectHandle.
	// We ASSUME we can store object pointer in the mxUINT32 element of mxArray.
	mxArray* to_mex_handle() {
		mxArray* handle = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
		*reinterpret_cast<ObjectHandle<T>**> (mxGetPr(handle)) = this;
		return handle;
	}

	string type_name() const {
		return type->name();
	}

	// Convert mxArray (passed to mex-function) to an ObjectHandle<T>.
	// Import a handle from MatLab as a mxArray of UINT32. Check that
	// it is actually a pointer to an ObjectHandle<T>.
	static ObjectHandle* from_mex_handle(const mxArray* handle) {
		if (mxGetClassID(handle) != mxUINT32OR64_CLASS || mxIsComplex(handle)
				|| mxGetM(handle) != 1 || mxGetN(handle) != 1) error(
				"Parameter is not an ObjectHandle type.");

		// We *assume* we can store ObjectHandle<T> pointer in the mxUINT32 of handle
		ObjectHandle* obj = *reinterpret_cast<ObjectHandle**> (mxGetPr(handle));

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
void delete_shared_ptr(const mxArray* obj, const string& className) {
  mxArray* mxh = mxGetProperty(obj,0,"self");
  ObjectHandle<Class>* handle = ObjectHandle<Class>::from_mex_handle(mxh);
  delete handle;
}

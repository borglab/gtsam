#include <gtwrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>



typedef MyTemplate<gtsam::Point2> MyTemplatePoint2;
typedef MyTemplate<gtsam::Matrix> MyTemplateMatrix;

typedef std::set<boost::shared_ptr<MyBase>*> Collector_MyBase;
static Collector_MyBase collector_MyBase;
typedef std::set<boost::shared_ptr<MyTemplatePoint2>*> Collector_MyTemplatePoint2;
static Collector_MyTemplatePoint2 collector_MyTemplatePoint2;
typedef std::set<boost::shared_ptr<MyTemplateMatrix>*> Collector_MyTemplateMatrix;
static Collector_MyTemplateMatrix collector_MyTemplateMatrix;
typedef std::set<boost::shared_ptr<ForwardKinematicsFactor>*> Collector_ForwardKinematicsFactor;
static Collector_ForwardKinematicsFactor collector_ForwardKinematicsFactor;


void _deleteAllObjects()
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  bool anyDeleted = false;
  { for(Collector_MyBase::iterator iter = collector_MyBase.begin();
      iter != collector_MyBase.end(); ) {
    delete *iter;
    collector_MyBase.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_MyTemplatePoint2::iterator iter = collector_MyTemplatePoint2.begin();
      iter != collector_MyTemplatePoint2.end(); ) {
    delete *iter;
    collector_MyTemplatePoint2.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_MyTemplateMatrix::iterator iter = collector_MyTemplateMatrix.begin();
      iter != collector_MyTemplateMatrix.end(); ) {
    delete *iter;
    collector_MyTemplateMatrix.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_ForwardKinematicsFactor::iterator iter = collector_ForwardKinematicsFactor.begin();
      iter != collector_ForwardKinematicsFactor.end(); ) {
    delete *iter;
    collector_ForwardKinematicsFactor.erase(iter++);
    anyDeleted = true;
  } }

  if(anyDeleted)
    cout <<
      "WARNING:  Wrap modules with variables in the workspace have been reloaded due to\n"
      "calling destructors, call 'clear all' again if you plan to now recompile a wrap\n"
      "module, so that your recompiled module is used instead of the old one." << endl;
  std::cout.rdbuf(outbuf);
}

void _inheritance_RTTIRegister() {
  const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_inheritance_rttiRegistry_created");
  if(!alreadyCreated) {
    std::map<std::string, std::string> types;

    types.insert(std::make_pair(typeid(MyBase).name(), "MyBase"));
    types.insert(std::make_pair(typeid(MyTemplatePoint2).name(), "MyTemplatePoint2"));
    types.insert(std::make_pair(typeid(MyTemplateMatrix).name(), "MyTemplateMatrix"));
    types.insert(std::make_pair(typeid(ForwardKinematicsFactor).name(), "ForwardKinematicsFactor"));


    mxArray *registry = mexGetVariable("global", "gtsamwrap_rttiRegistry");
    if(!registry)
      registry = mxCreateStructMatrix(1, 1, 0, NULL);
    typedef std::pair<std::string, std::string> StringPair;
    for(const StringPair& rtti_matlab: types) {
      int fieldId = mxAddField(registry, rtti_matlab.first.c_str());
      if(fieldId < 0) {
        mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
      }
      mxArray *matlabName = mxCreateString(rtti_matlab.second.c_str());
      mxSetFieldByNumber(registry, 0, fieldId, matlabName);
    }
    if(mexPutVariable("global", "gtsamwrap_rttiRegistry", registry) != 0) {
      mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
    }
    mxDestroyArray(registry);

    mxArray *newAlreadyCreated = mxCreateNumericMatrix(0, 0, mxINT8_CLASS, mxREAL);
    if(mexPutVariable("global", "gtsam_geometry_rttiRegistry_created", newAlreadyCreated) != 0) {
      mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
    }
    mxDestroyArray(newAlreadyCreated);
  }
}

void MyBase_collectorInsertAndMakeBase_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyBase> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyBase.insert(self);
}

void MyBase_upcastFromVoid_1(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyBase> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<MyBase>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void MyBase_deconstructor_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<MyBase> Shared;
  checkArguments("delete_MyBase",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_MyBase::iterator item;
  item = collector_MyBase.find(self);
  if(item != collector_MyBase.end()) {
    delete self;
    collector_MyBase.erase(item);
  }
}

void MyTemplatePoint2_collectorInsertAndMakeBase_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Point2>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyTemplatePoint2.insert(self);

  typedef boost::shared_ptr<MyBase> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void MyTemplatePoint2_upcastFromVoid_4(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Point2>> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<MyTemplate<gtsam::Point2>>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void MyTemplatePoint2_constructor_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Point2>> Shared;

  Shared *self = new Shared(new MyTemplate<gtsam::Point2>());
  collector_MyTemplatePoint2.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<MyBase> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void MyTemplatePoint2_deconstructor_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<MyTemplate<gtsam::Point2>> Shared;
  checkArguments("delete_MyTemplatePoint2",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_MyTemplatePoint2::iterator item;
  item = collector_MyTemplatePoint2.find(self);
  if(item != collector_MyTemplatePoint2.end()) {
    delete self;
    collector_MyTemplatePoint2.erase(item);
  }
}

void MyTemplatePoint2_accept_T_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  obj->accept_T(value);
}

void MyTemplatePoint2_accept_Tptr_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  obj->accept_Tptr(value);
}

void MyTemplatePoint2_create_MixedPtrs_9(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_MixedPtrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  auto pairResult = obj->create_MixedPtrs();
  out[0] = wrap< Point2 >(pairResult.first);
  {
  boost::shared_ptr<Point2> shared(pairResult.second);
  out[1] = wrap_shared_ptr(shared,"Point2");
  }
}

void MyTemplatePoint2_create_ptrs_10(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_ptrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  auto pairResult = obj->create_ptrs();
  {
  boost::shared_ptr<Point2> shared(pairResult.first);
  out[0] = wrap_shared_ptr(shared,"Point2");
  }
  {
  boost::shared_ptr<Point2> shared(pairResult.second);
  out[1] = wrap_shared_ptr(shared,"Point2");
  }
}

void MyTemplatePoint2_return_T_11(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  out[0] = wrap< Point2 >(obj->return_T(value));
}

void MyTemplatePoint2_return_Tptr_12(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  {
  boost::shared_ptr<Point2> shared(obj->return_Tptr(value));
  out[0] = wrap_shared_ptr(shared,"Point2");
  }
}

void MyTemplatePoint2_return_ptrs_13(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_ptrs",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 p1 = unwrap< Point2 >(in[1]);
  Point2 p2 = unwrap< Point2 >(in[2]);
  auto pairResult = obj->return_ptrs(p1,p2);
  {
  boost::shared_ptr<Point2> shared(pairResult.first);
  out[0] = wrap_shared_ptr(shared,"Point2");
  }
  {
  boost::shared_ptr<Point2> shared(pairResult.second);
  out[1] = wrap_shared_ptr(shared,"Point2");
  }
}

void MyTemplatePoint2_templatedMethod_14(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodMatrix",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Matrix t = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->templatedMethod<gtsam::Matrix>(t));
}

void MyTemplatePoint2_templatedMethod_15(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 t = unwrap< Point2 >(in[1]);
  out[0] = wrap< Point2 >(obj->templatedMethod<gtsam::Point2>(t));
}

void MyTemplatePoint2_templatedMethod_16(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint3",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point3 t = unwrap< Point3 >(in[1]);
  out[0] = wrap< Point3 >(obj->templatedMethod<gtsam::Point3>(t));
}

void MyTemplatePoint2_templatedMethod_17(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodVector",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Vector t = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->templatedMethod<gtsam::Vector>(t));
}

void MyTemplatePoint2_Level_18(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MyTemplatePoint2.Level",nargout,nargin,1);
  Point2 K = unwrap< Point2 >(in[0]);
  out[0] = wrap_shared_ptr(boost::make_shared<MyTemplate<Point2>>(MyTemplate<gtsam::Point2>::Level(K)),"MyTemplatePoint2", false);
}

void MyTemplateMatrix_collectorInsertAndMakeBase_19(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Matrix>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyTemplateMatrix.insert(self);

  typedef boost::shared_ptr<MyBase> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void MyTemplateMatrix_upcastFromVoid_20(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Matrix>> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<MyTemplate<gtsam::Matrix>>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void MyTemplateMatrix_constructor_21(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Matrix>> Shared;

  Shared *self = new Shared(new MyTemplate<gtsam::Matrix>());
  collector_MyTemplateMatrix.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;

  typedef boost::shared_ptr<MyBase> SharedBase;
  out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
}

void MyTemplateMatrix_deconstructor_22(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<MyTemplate<gtsam::Matrix>> Shared;
  checkArguments("delete_MyTemplateMatrix",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_MyTemplateMatrix::iterator item;
  item = collector_MyTemplateMatrix.find(self);
  if(item != collector_MyTemplateMatrix.end()) {
    delete self;
    collector_MyTemplateMatrix.erase(item);
  }
}

void MyTemplateMatrix_accept_T_23(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  obj->accept_T(value);
}

void MyTemplateMatrix_accept_Tptr_24(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  obj->accept_Tptr(value);
}

void MyTemplateMatrix_create_MixedPtrs_25(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_MixedPtrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  auto pairResult = obj->create_MixedPtrs();
  out[0] = wrap< Matrix >(pairResult.first);
  {
  boost::shared_ptr<Matrix> shared(pairResult.second);
  out[1] = wrap_shared_ptr(shared,"Matrix");
  }
}

void MyTemplateMatrix_create_ptrs_26(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_ptrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  auto pairResult = obj->create_ptrs();
  {
  boost::shared_ptr<Matrix> shared(pairResult.first);
  out[0] = wrap_shared_ptr(shared,"Matrix");
  }
  {
  boost::shared_ptr<Matrix> shared(pairResult.second);
  out[1] = wrap_shared_ptr(shared,"Matrix");
  }
}

void MyTemplateMatrix_return_T_27(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->return_T(value));
}

void MyTemplateMatrix_return_Tptr_28(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  {
  boost::shared_ptr<Matrix> shared(obj->return_Tptr(value));
  out[0] = wrap_shared_ptr(shared,"Matrix");
  }
}

void MyTemplateMatrix_return_ptrs_29(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_ptrs",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix p1 = unwrap< Matrix >(in[1]);
  Matrix p2 = unwrap< Matrix >(in[2]);
  auto pairResult = obj->return_ptrs(p1,p2);
  {
  boost::shared_ptr<Matrix> shared(pairResult.first);
  out[0] = wrap_shared_ptr(shared,"Matrix");
  }
  {
  boost::shared_ptr<Matrix> shared(pairResult.second);
  out[1] = wrap_shared_ptr(shared,"Matrix");
  }
}

void MyTemplateMatrix_templatedMethod_30(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodMatrix",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix t = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->templatedMethod<gtsam::Matrix>(t));
}

void MyTemplateMatrix_templatedMethod_31(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Point2 t = unwrap< Point2 >(in[1]);
  out[0] = wrap< Point2 >(obj->templatedMethod<gtsam::Point2>(t));
}

void MyTemplateMatrix_templatedMethod_32(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint3",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Point3 t = unwrap< Point3 >(in[1]);
  out[0] = wrap< Point3 >(obj->templatedMethod<gtsam::Point3>(t));
}

void MyTemplateMatrix_templatedMethod_33(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodVector",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Vector t = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->templatedMethod<gtsam::Vector>(t));
}

void MyTemplateMatrix_Level_34(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MyTemplateMatrix.Level",nargout,nargin,1);
  Matrix K = unwrap< Matrix >(in[0]);
  out[0] = wrap_shared_ptr(boost::make_shared<MyTemplate<Matrix>>(MyTemplate<gtsam::Matrix>::Level(K)),"MyTemplateMatrix", false);
}

void ForwardKinematicsFactor_collectorInsertAndMakeBase_35(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<ForwardKinematicsFactor> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_ForwardKinematicsFactor.insert(self);

  typedef boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void ForwardKinematicsFactor_upcastFromVoid_36(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<ForwardKinematicsFactor> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<ForwardKinematicsFactor>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void ForwardKinematicsFactor_deconstructor_37(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<ForwardKinematicsFactor> Shared;
  checkArguments("delete_ForwardKinematicsFactor",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_ForwardKinematicsFactor::iterator item;
  item = collector_ForwardKinematicsFactor.find(self);
  if(item != collector_ForwardKinematicsFactor.end()) {
    delete self;
    collector_ForwardKinematicsFactor.erase(item);
  }
}


void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  _inheritance_RTTIRegister();

  int id = unwrap<int>(in[0]);

  try {
    switch(id) {
    case 0:
      MyBase_collectorInsertAndMakeBase_0(nargout, out, nargin-1, in+1);
      break;
    case 1:
      MyBase_upcastFromVoid_1(nargout, out, nargin-1, in+1);
      break;
    case 2:
      MyBase_deconstructor_2(nargout, out, nargin-1, in+1);
      break;
    case 3:
      MyTemplatePoint2_collectorInsertAndMakeBase_3(nargout, out, nargin-1, in+1);
      break;
    case 4:
      MyTemplatePoint2_upcastFromVoid_4(nargout, out, nargin-1, in+1);
      break;
    case 5:
      MyTemplatePoint2_constructor_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      MyTemplatePoint2_deconstructor_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      MyTemplatePoint2_accept_T_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      MyTemplatePoint2_accept_Tptr_8(nargout, out, nargin-1, in+1);
      break;
    case 9:
      MyTemplatePoint2_create_MixedPtrs_9(nargout, out, nargin-1, in+1);
      break;
    case 10:
      MyTemplatePoint2_create_ptrs_10(nargout, out, nargin-1, in+1);
      break;
    case 11:
      MyTemplatePoint2_return_T_11(nargout, out, nargin-1, in+1);
      break;
    case 12:
      MyTemplatePoint2_return_Tptr_12(nargout, out, nargin-1, in+1);
      break;
    case 13:
      MyTemplatePoint2_return_ptrs_13(nargout, out, nargin-1, in+1);
      break;
    case 14:
      MyTemplatePoint2_templatedMethod_14(nargout, out, nargin-1, in+1);
      break;
    case 15:
      MyTemplatePoint2_templatedMethod_15(nargout, out, nargin-1, in+1);
      break;
    case 16:
      MyTemplatePoint2_templatedMethod_16(nargout, out, nargin-1, in+1);
      break;
    case 17:
      MyTemplatePoint2_templatedMethod_17(nargout, out, nargin-1, in+1);
      break;
    case 18:
      MyTemplatePoint2_Level_18(nargout, out, nargin-1, in+1);
      break;
    case 19:
      MyTemplateMatrix_collectorInsertAndMakeBase_19(nargout, out, nargin-1, in+1);
      break;
    case 20:
      MyTemplateMatrix_upcastFromVoid_20(nargout, out, nargin-1, in+1);
      break;
    case 21:
      MyTemplateMatrix_constructor_21(nargout, out, nargin-1, in+1);
      break;
    case 22:
      MyTemplateMatrix_deconstructor_22(nargout, out, nargin-1, in+1);
      break;
    case 23:
      MyTemplateMatrix_accept_T_23(nargout, out, nargin-1, in+1);
      break;
    case 24:
      MyTemplateMatrix_accept_Tptr_24(nargout, out, nargin-1, in+1);
      break;
    case 25:
      MyTemplateMatrix_create_MixedPtrs_25(nargout, out, nargin-1, in+1);
      break;
    case 26:
      MyTemplateMatrix_create_ptrs_26(nargout, out, nargin-1, in+1);
      break;
    case 27:
      MyTemplateMatrix_return_T_27(nargout, out, nargin-1, in+1);
      break;
    case 28:
      MyTemplateMatrix_return_Tptr_28(nargout, out, nargin-1, in+1);
      break;
    case 29:
      MyTemplateMatrix_return_ptrs_29(nargout, out, nargin-1, in+1);
      break;
    case 30:
      MyTemplateMatrix_templatedMethod_30(nargout, out, nargin-1, in+1);
      break;
    case 31:
      MyTemplateMatrix_templatedMethod_31(nargout, out, nargin-1, in+1);
      break;
    case 32:
      MyTemplateMatrix_templatedMethod_32(nargout, out, nargin-1, in+1);
      break;
    case 33:
      MyTemplateMatrix_templatedMethod_33(nargout, out, nargin-1, in+1);
      break;
    case 34:
      MyTemplateMatrix_Level_34(nargout, out, nargin-1, in+1);
      break;
    case 35:
      ForwardKinematicsFactor_collectorInsertAndMakeBase_35(nargout, out, nargin-1, in+1);
      break;
    case 36:
      ForwardKinematicsFactor_upcastFromVoid_36(nargout, out, nargin-1, in+1);
      break;
    case 37:
      ForwardKinematicsFactor_deconstructor_37(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

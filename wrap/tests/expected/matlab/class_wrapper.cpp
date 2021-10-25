#include <gtwrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>

#include <folder/path/to/Test.h>

typedef Fun<double> FunDouble;
typedef PrimitiveRef<double> PrimitiveRefDouble;
typedef MyVector<3> MyVector3;
typedef MyVector<12> MyVector12;
typedef MultipleTemplates<int, double> MultipleTemplatesIntDouble;
typedef MultipleTemplates<int, float> MultipleTemplatesIntFloat;
typedef MyFactor<gtsam::Pose2, gtsam::Matrix> MyFactorPosePoint2;

typedef std::set<boost::shared_ptr<FunRange>*> Collector_FunRange;
static Collector_FunRange collector_FunRange;
typedef std::set<boost::shared_ptr<FunDouble>*> Collector_FunDouble;
static Collector_FunDouble collector_FunDouble;
typedef std::set<boost::shared_ptr<Test>*> Collector_Test;
static Collector_Test collector_Test;
typedef std::set<boost::shared_ptr<PrimitiveRefDouble>*> Collector_PrimitiveRefDouble;
static Collector_PrimitiveRefDouble collector_PrimitiveRefDouble;
typedef std::set<boost::shared_ptr<MyVector3>*> Collector_MyVector3;
static Collector_MyVector3 collector_MyVector3;
typedef std::set<boost::shared_ptr<MyVector12>*> Collector_MyVector12;
static Collector_MyVector12 collector_MyVector12;
typedef std::set<boost::shared_ptr<MultipleTemplatesIntDouble>*> Collector_MultipleTemplatesIntDouble;
static Collector_MultipleTemplatesIntDouble collector_MultipleTemplatesIntDouble;
typedef std::set<boost::shared_ptr<MultipleTemplatesIntFloat>*> Collector_MultipleTemplatesIntFloat;
static Collector_MultipleTemplatesIntFloat collector_MultipleTemplatesIntFloat;
typedef std::set<boost::shared_ptr<ForwardKinematics>*> Collector_ForwardKinematics;
static Collector_ForwardKinematics collector_ForwardKinematics;
typedef std::set<boost::shared_ptr<TemplatedConstructor>*> Collector_TemplatedConstructor;
static Collector_TemplatedConstructor collector_TemplatedConstructor;
typedef std::set<boost::shared_ptr<MyFactorPosePoint2>*> Collector_MyFactorPosePoint2;
static Collector_MyFactorPosePoint2 collector_MyFactorPosePoint2;


void _deleteAllObjects()
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  bool anyDeleted = false;
  { for(Collector_FunRange::iterator iter = collector_FunRange.begin();
      iter != collector_FunRange.end(); ) {
    delete *iter;
    collector_FunRange.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_FunDouble::iterator iter = collector_FunDouble.begin();
      iter != collector_FunDouble.end(); ) {
    delete *iter;
    collector_FunDouble.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_Test::iterator iter = collector_Test.begin();
      iter != collector_Test.end(); ) {
    delete *iter;
    collector_Test.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_PrimitiveRefDouble::iterator iter = collector_PrimitiveRefDouble.begin();
      iter != collector_PrimitiveRefDouble.end(); ) {
    delete *iter;
    collector_PrimitiveRefDouble.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_MyVector3::iterator iter = collector_MyVector3.begin();
      iter != collector_MyVector3.end(); ) {
    delete *iter;
    collector_MyVector3.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_MyVector12::iterator iter = collector_MyVector12.begin();
      iter != collector_MyVector12.end(); ) {
    delete *iter;
    collector_MyVector12.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_MultipleTemplatesIntDouble::iterator iter = collector_MultipleTemplatesIntDouble.begin();
      iter != collector_MultipleTemplatesIntDouble.end(); ) {
    delete *iter;
    collector_MultipleTemplatesIntDouble.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_MultipleTemplatesIntFloat::iterator iter = collector_MultipleTemplatesIntFloat.begin();
      iter != collector_MultipleTemplatesIntFloat.end(); ) {
    delete *iter;
    collector_MultipleTemplatesIntFloat.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_ForwardKinematics::iterator iter = collector_ForwardKinematics.begin();
      iter != collector_ForwardKinematics.end(); ) {
    delete *iter;
    collector_ForwardKinematics.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_TemplatedConstructor::iterator iter = collector_TemplatedConstructor.begin();
      iter != collector_TemplatedConstructor.end(); ) {
    delete *iter;
    collector_TemplatedConstructor.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_MyFactorPosePoint2::iterator iter = collector_MyFactorPosePoint2.begin();
      iter != collector_MyFactorPosePoint2.end(); ) {
    delete *iter;
    collector_MyFactorPosePoint2.erase(iter++);
    anyDeleted = true;
  } }

  if(anyDeleted)
    cout <<
      "WARNING:  Wrap modules with variables in the workspace have been reloaded due to\n"
      "calling destructors, call 'clear all' again if you plan to now recompile a wrap\n"
      "module, so that your recompiled module is used instead of the old one." << endl;
  std::cout.rdbuf(outbuf);
}

void _class_RTTIRegister() {
  const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_class_rttiRegistry_created");
  if(!alreadyCreated) {
    std::map<std::string, std::string> types;



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

void FunRange_collectorInsertAndMakeBase_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<FunRange> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_FunRange.insert(self);
}

void FunRange_constructor_1(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<FunRange> Shared;

  Shared *self = new Shared(new FunRange());
  collector_FunRange.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void FunRange_deconstructor_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<FunRange> Shared;
  checkArguments("delete_FunRange",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_FunRange::iterator item;
  item = collector_FunRange.find(self);
  if(item != collector_FunRange.end()) {
    delete self;
    collector_FunRange.erase(item);
  }
}

void FunRange_range_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("range",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<FunRange>(in[0], "ptr_FunRange");
  double d = unwrap< double >(in[1]);
  out[0] = wrap_shared_ptr(boost::make_shared<FunRange>(obj->range(d)),"FunRange", false);
}

void FunRange_create_4(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("FunRange.create",nargout,nargin,0);
  out[0] = wrap_shared_ptr(boost::make_shared<FunRange>(FunRange::create()),"FunRange", false);
}

void FunDouble_collectorInsertAndMakeBase_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<Fun<double>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_FunDouble.insert(self);
}

void FunDouble_deconstructor_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<Fun<double>> Shared;
  checkArguments("delete_FunDouble",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_FunDouble::iterator item;
  item = collector_FunDouble.find(self);
  if(item != collector_FunDouble.end()) {
    delete self;
    collector_FunDouble.erase(item);
  }
}

void FunDouble_multiTemplatedMethod_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("multiTemplatedMethodStringSize_t",nargout,nargin-1,3);
  auto obj = unwrap_shared_ptr<Fun<double>>(in[0], "ptr_FunDouble");
  double d = unwrap< double >(in[1]);
  string t = unwrap< string >(in[2]);
  size_t u = unwrap< size_t >(in[3]);
  out[0] = wrap_shared_ptr(boost::make_shared<Fun<double>>(obj->multiTemplatedMethod<string,size_t>(d,t,u)),"Fun<double>", false);
}

void FunDouble_templatedMethod_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodString",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<Fun<double>>(in[0], "ptr_FunDouble");
  double d = unwrap< double >(in[1]);
  string t = unwrap< string >(in[2]);
  out[0] = wrap_shared_ptr(boost::make_shared<Fun<double>>(obj->templatedMethod<string>(d,t)),"Fun<double>", false);
}

void FunDouble_staticMethodWithThis_9(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("FunDouble.staticMethodWithThis",nargout,nargin,0);
  out[0] = wrap_shared_ptr(boost::make_shared<Fun<double>>(Fun<double>::staticMethodWithThis()),"Fundouble", false);
}

void FunDouble_templatedStaticMethodInt_10(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("FunDouble.templatedStaticMethodInt",nargout,nargin,1);
  int m = unwrap< int >(in[0]);
  out[0] = wrap< double >(Fun<double>::templatedStaticMethodInt(m));
}

void Test_collectorInsertAndMakeBase_11(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<Test> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_Test.insert(self);
}

void Test_constructor_12(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<Test> Shared;

  Shared *self = new Shared(new Test());
  collector_Test.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void Test_constructor_13(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<Test> Shared;

  double a = unwrap< double >(in[0]);
  Matrix b = unwrap< Matrix >(in[1]);
  Shared *self = new Shared(new Test(a,b));
  collector_Test.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void Test_deconstructor_14(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<Test> Shared;
  checkArguments("delete_Test",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_Test::iterator item;
  item = collector_Test.find(self);
  if(item != collector_Test.end()) {
    delete self;
    collector_Test.erase(item);
  }
}

void Test_arg_EigenConstRef_15(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("arg_EigenConstRef",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Matrix value = unwrap< Matrix >(in[1]);
  obj->arg_EigenConstRef(value);
}

void Test_create_MixedPtrs_16(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_MixedPtrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  auto pairResult = obj->create_MixedPtrs();
  out[0] = wrap_shared_ptr(boost::make_shared<Test>(pairResult.first),"Test", false);
  out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
}

void Test_create_ptrs_17(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_ptrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  auto pairResult = obj->create_ptrs();
  out[0] = wrap_shared_ptr(pairResult.first,"Test", false);
  out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
}

void Test_get_container_18(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("get_container",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  out[0] = wrap_shared_ptr(boost::make_shared<std::vector<testing::Test>>(obj->get_container()),"std.vectorTest", false);
}

void Test_lambda_19(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("lambda",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  obj->lambda();
}

void Test_print_20(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("print",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  obj->print();
}

void Test_return_Point2Ptr_21(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Point2Ptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  bool value = unwrap< bool >(in[1]);
  {
  boost::shared_ptr<Point2> shared(obj->return_Point2Ptr(value));
  out[0] = wrap_shared_ptr(shared,"Point2");
  }
}

void Test_return_Test_22(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Test",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<Test> value = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  out[0] = wrap_shared_ptr(boost::make_shared<Test>(obj->return_Test(value)),"Test", false);
}

void Test_return_TestPtr_23(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_TestPtr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<Test> value = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  out[0] = wrap_shared_ptr(obj->return_TestPtr(value),"Test", false);
}

void Test_return_bool_24(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_bool",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  bool value = unwrap< bool >(in[1]);
  out[0] = wrap< bool >(obj->return_bool(value));
}

void Test_return_double_25(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_double",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  double value = unwrap< double >(in[1]);
  out[0] = wrap< double >(obj->return_double(value));
}

void Test_return_field_26(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_field",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Test& t = *unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  out[0] = wrap< bool >(obj->return_field(t));
}

void Test_return_int_27(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_int",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  int value = unwrap< int >(in[1]);
  out[0] = wrap< int >(obj->return_int(value));
}

void Test_return_matrix1_28(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_matrix1",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Matrix value = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->return_matrix1(value));
}

void Test_return_matrix2_29(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_matrix2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Matrix value = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->return_matrix2(value));
}

void Test_return_pair_30(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_pair",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector v = unwrap< Vector >(in[1]);
  Matrix A = unwrap< Matrix >(in[2]);
  auto pairResult = obj->return_pair(v,A);
  out[0] = wrap< Vector >(pairResult.first);
  out[1] = wrap< Matrix >(pairResult.second);
}

void Test_return_pair_31(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_pair",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector v = unwrap< Vector >(in[1]);
  auto pairResult = obj->return_pair(v);
  out[0] = wrap< Vector >(pairResult.first);
  out[1] = wrap< Matrix >(pairResult.second);
}

void Test_return_ptrs_32(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_ptrs",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<Test> p1 = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  boost::shared_ptr<Test> p2 = unwrap_shared_ptr< Test >(in[2], "ptr_Test");
  auto pairResult = obj->return_ptrs(p1,p2);
  out[0] = wrap_shared_ptr(pairResult.first,"Test", false);
  out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
}

void Test_return_size_t_33(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_size_t",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  size_t value = unwrap< size_t >(in[1]);
  out[0] = wrap< size_t >(obj->return_size_t(value));
}

void Test_return_string_34(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_string",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  string value = unwrap< string >(in[1]);
  out[0] = wrap< string >(obj->return_string(value));
}

void Test_return_vector1_35(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_vector1",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector value = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->return_vector1(value));
}

void Test_return_vector2_36(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_vector2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector value = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->return_vector2(value));
}

void Test_set_container_37(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("set_container",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<std::vector<testing::Test>> container = unwrap_shared_ptr< std::vector<testing::Test> >(in[1], "ptr_stdvectorTest");
  obj->set_container(*container);
}

void Test_set_container_38(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("set_container",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<std::vector<testing::Test>> container = unwrap_shared_ptr< std::vector<testing::Test> >(in[1], "ptr_stdvectorTest");
  obj->set_container(*container);
}

void Test_set_container_39(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("set_container",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<std::vector<testing::Test>> container = unwrap_shared_ptr< std::vector<testing::Test> >(in[1], "ptr_stdvectorTest");
  obj->set_container(*container);
}

void PrimitiveRefDouble_collectorInsertAndMakeBase_40(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<PrimitiveRef<double>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_PrimitiveRefDouble.insert(self);
}

void PrimitiveRefDouble_constructor_41(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<PrimitiveRef<double>> Shared;

  Shared *self = new Shared(new PrimitiveRef<double>());
  collector_PrimitiveRefDouble.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void PrimitiveRefDouble_deconstructor_42(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<PrimitiveRef<double>> Shared;
  checkArguments("delete_PrimitiveRefDouble",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_PrimitiveRefDouble::iterator item;
  item = collector_PrimitiveRefDouble.find(self);
  if(item != collector_PrimitiveRefDouble.end()) {
    delete self;
    collector_PrimitiveRefDouble.erase(item);
  }
}

void PrimitiveRefDouble_Brutal_43(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("PrimitiveRefDouble.Brutal",nargout,nargin,1);
  double t = unwrap< double >(in[0]);
  out[0] = wrap_shared_ptr(boost::make_shared<PrimitiveRef<double>>(PrimitiveRef<double>::Brutal(t)),"PrimitiveRefdouble", false);
}

void MyVector3_collectorInsertAndMakeBase_44(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<3>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyVector3.insert(self);
}

void MyVector3_constructor_45(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<3>> Shared;

  Shared *self = new Shared(new MyVector<3>());
  collector_MyVector3.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void MyVector3_deconstructor_46(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<MyVector<3>> Shared;
  checkArguments("delete_MyVector3",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_MyVector3::iterator item;
  item = collector_MyVector3.find(self);
  if(item != collector_MyVector3.end()) {
    delete self;
    collector_MyVector3.erase(item);
  }
}

void MyVector12_collectorInsertAndMakeBase_47(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<12>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyVector12.insert(self);
}

void MyVector12_constructor_48(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<12>> Shared;

  Shared *self = new Shared(new MyVector<12>());
  collector_MyVector12.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void MyVector12_deconstructor_49(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<MyVector<12>> Shared;
  checkArguments("delete_MyVector12",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_MyVector12::iterator item;
  item = collector_MyVector12.find(self);
  if(item != collector_MyVector12.end()) {
    delete self;
    collector_MyVector12.erase(item);
  }
}

void MultipleTemplatesIntDouble_collectorInsertAndMakeBase_50(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MultipleTemplates<int, double>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MultipleTemplatesIntDouble.insert(self);
}

void MultipleTemplatesIntDouble_deconstructor_51(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<MultipleTemplates<int, double>> Shared;
  checkArguments("delete_MultipleTemplatesIntDouble",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_MultipleTemplatesIntDouble::iterator item;
  item = collector_MultipleTemplatesIntDouble.find(self);
  if(item != collector_MultipleTemplatesIntDouble.end()) {
    delete self;
    collector_MultipleTemplatesIntDouble.erase(item);
  }
}

void MultipleTemplatesIntFloat_collectorInsertAndMakeBase_52(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MultipleTemplates<int, float>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MultipleTemplatesIntFloat.insert(self);
}

void MultipleTemplatesIntFloat_deconstructor_53(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<MultipleTemplates<int, float>> Shared;
  checkArguments("delete_MultipleTemplatesIntFloat",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_MultipleTemplatesIntFloat::iterator item;
  item = collector_MultipleTemplatesIntFloat.find(self);
  if(item != collector_MultipleTemplatesIntFloat.end()) {
    delete self;
    collector_MultipleTemplatesIntFloat.erase(item);
  }
}

void ForwardKinematics_collectorInsertAndMakeBase_54(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<ForwardKinematics> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_ForwardKinematics.insert(self);
}

void ForwardKinematics_constructor_55(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<ForwardKinematics> Shared;

  gtdynamics::Robot& robot = *unwrap_shared_ptr< gtdynamics::Robot >(in[0], "ptr_gtdynamicsRobot");
  string& start_link_name = *unwrap_shared_ptr< string >(in[1], "ptr_string");
  string& end_link_name = *unwrap_shared_ptr< string >(in[2], "ptr_string");
  gtsam::Values& joint_angles = *unwrap_shared_ptr< gtsam::Values >(in[3], "ptr_gtsamValues");
  gtsam::Pose3& l2Tp = *unwrap_shared_ptr< gtsam::Pose3 >(in[4], "ptr_gtsamPose3");
  Shared *self = new Shared(new ForwardKinematics(robot,start_link_name,end_link_name,joint_angles,l2Tp));
  collector_ForwardKinematics.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void ForwardKinematics_deconstructor_56(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<ForwardKinematics> Shared;
  checkArguments("delete_ForwardKinematics",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_ForwardKinematics::iterator item;
  item = collector_ForwardKinematics.find(self);
  if(item != collector_ForwardKinematics.end()) {
    delete self;
    collector_ForwardKinematics.erase(item);
  }
}

void TemplatedConstructor_collectorInsertAndMakeBase_57(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_TemplatedConstructor.insert(self);
}

void TemplatedConstructor_constructor_58(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  Shared *self = new Shared(new TemplatedConstructor());
  collector_TemplatedConstructor.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void TemplatedConstructor_constructor_59(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  string& arg = *unwrap_shared_ptr< string >(in[0], "ptr_string");
  Shared *self = new Shared(new TemplatedConstructor(arg));
  collector_TemplatedConstructor.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void TemplatedConstructor_constructor_60(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  int arg = unwrap< int >(in[0]);
  Shared *self = new Shared(new TemplatedConstructor(arg));
  collector_TemplatedConstructor.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void TemplatedConstructor_constructor_61(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  double arg = unwrap< double >(in[0]);
  Shared *self = new Shared(new TemplatedConstructor(arg));
  collector_TemplatedConstructor.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void TemplatedConstructor_deconstructor_62(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<TemplatedConstructor> Shared;
  checkArguments("delete_TemplatedConstructor",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_TemplatedConstructor::iterator item;
  item = collector_TemplatedConstructor.find(self);
  if(item != collector_TemplatedConstructor.end()) {
    delete self;
    collector_TemplatedConstructor.erase(item);
  }
}

void MyFactorPosePoint2_collectorInsertAndMakeBase_63(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyFactorPosePoint2.insert(self);
}

void MyFactorPosePoint2_constructor_64(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>> Shared;

  size_t key1 = unwrap< size_t >(in[0]);
  size_t key2 = unwrap< size_t >(in[1]);
  double measured = unwrap< double >(in[2]);
  boost::shared_ptr<gtsam::noiseModel::Base> noiseModel = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[3], "ptr_gtsamnoiseModelBase");
  Shared *self = new Shared(new MyFactor<gtsam::Pose2, gtsam::Matrix>(key1,key2,measured,noiseModel));
  collector_MyFactorPosePoint2.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void MyFactorPosePoint2_deconstructor_65(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>> Shared;
  checkArguments("delete_MyFactorPosePoint2",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_MyFactorPosePoint2::iterator item;
  item = collector_MyFactorPosePoint2.find(self);
  if(item != collector_MyFactorPosePoint2.end()) {
    delete self;
    collector_MyFactorPosePoint2.erase(item);
  }
}

void MyFactorPosePoint2_print_66(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("print",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>>(in[0], "ptr_MyFactorPosePoint2");
  string& s = *unwrap_shared_ptr< string >(in[1], "ptr_string");
  gtsam::KeyFormatter& keyFormatter = *unwrap_shared_ptr< gtsam::KeyFormatter >(in[2], "ptr_gtsamKeyFormatter");
  obj->print(s,keyFormatter);
}


void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  _class_RTTIRegister();

  int id = unwrap<int>(in[0]);

  try {
    switch(id) {
    case 0:
      FunRange_collectorInsertAndMakeBase_0(nargout, out, nargin-1, in+1);
      break;
    case 1:
      FunRange_constructor_1(nargout, out, nargin-1, in+1);
      break;
    case 2:
      FunRange_deconstructor_2(nargout, out, nargin-1, in+1);
      break;
    case 3:
      FunRange_range_3(nargout, out, nargin-1, in+1);
      break;
    case 4:
      FunRange_create_4(nargout, out, nargin-1, in+1);
      break;
    case 5:
      FunDouble_collectorInsertAndMakeBase_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      FunDouble_deconstructor_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      FunDouble_multiTemplatedMethod_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      FunDouble_templatedMethod_8(nargout, out, nargin-1, in+1);
      break;
    case 9:
      FunDouble_staticMethodWithThis_9(nargout, out, nargin-1, in+1);
      break;
    case 10:
      FunDouble_templatedStaticMethodInt_10(nargout, out, nargin-1, in+1);
      break;
    case 11:
      Test_collectorInsertAndMakeBase_11(nargout, out, nargin-1, in+1);
      break;
    case 12:
      Test_constructor_12(nargout, out, nargin-1, in+1);
      break;
    case 13:
      Test_constructor_13(nargout, out, nargin-1, in+1);
      break;
    case 14:
      Test_deconstructor_14(nargout, out, nargin-1, in+1);
      break;
    case 15:
      Test_arg_EigenConstRef_15(nargout, out, nargin-1, in+1);
      break;
    case 16:
      Test_create_MixedPtrs_16(nargout, out, nargin-1, in+1);
      break;
    case 17:
      Test_create_ptrs_17(nargout, out, nargin-1, in+1);
      break;
    case 18:
      Test_get_container_18(nargout, out, nargin-1, in+1);
      break;
    case 19:
      Test_lambda_19(nargout, out, nargin-1, in+1);
      break;
    case 20:
      Test_print_20(nargout, out, nargin-1, in+1);
      break;
    case 21:
      Test_return_Point2Ptr_21(nargout, out, nargin-1, in+1);
      break;
    case 22:
      Test_return_Test_22(nargout, out, nargin-1, in+1);
      break;
    case 23:
      Test_return_TestPtr_23(nargout, out, nargin-1, in+1);
      break;
    case 24:
      Test_return_bool_24(nargout, out, nargin-1, in+1);
      break;
    case 25:
      Test_return_double_25(nargout, out, nargin-1, in+1);
      break;
    case 26:
      Test_return_field_26(nargout, out, nargin-1, in+1);
      break;
    case 27:
      Test_return_int_27(nargout, out, nargin-1, in+1);
      break;
    case 28:
      Test_return_matrix1_28(nargout, out, nargin-1, in+1);
      break;
    case 29:
      Test_return_matrix2_29(nargout, out, nargin-1, in+1);
      break;
    case 30:
      Test_return_pair_30(nargout, out, nargin-1, in+1);
      break;
    case 31:
      Test_return_pair_31(nargout, out, nargin-1, in+1);
      break;
    case 32:
      Test_return_ptrs_32(nargout, out, nargin-1, in+1);
      break;
    case 33:
      Test_return_size_t_33(nargout, out, nargin-1, in+1);
      break;
    case 34:
      Test_return_string_34(nargout, out, nargin-1, in+1);
      break;
    case 35:
      Test_return_vector1_35(nargout, out, nargin-1, in+1);
      break;
    case 36:
      Test_return_vector2_36(nargout, out, nargin-1, in+1);
      break;
    case 37:
      Test_set_container_37(nargout, out, nargin-1, in+1);
      break;
    case 38:
      Test_set_container_38(nargout, out, nargin-1, in+1);
      break;
    case 39:
      Test_set_container_39(nargout, out, nargin-1, in+1);
      break;
    case 40:
      PrimitiveRefDouble_collectorInsertAndMakeBase_40(nargout, out, nargin-1, in+1);
      break;
    case 41:
      PrimitiveRefDouble_constructor_41(nargout, out, nargin-1, in+1);
      break;
    case 42:
      PrimitiveRefDouble_deconstructor_42(nargout, out, nargin-1, in+1);
      break;
    case 43:
      PrimitiveRefDouble_Brutal_43(nargout, out, nargin-1, in+1);
      break;
    case 44:
      MyVector3_collectorInsertAndMakeBase_44(nargout, out, nargin-1, in+1);
      break;
    case 45:
      MyVector3_constructor_45(nargout, out, nargin-1, in+1);
      break;
    case 46:
      MyVector3_deconstructor_46(nargout, out, nargin-1, in+1);
      break;
    case 47:
      MyVector12_collectorInsertAndMakeBase_47(nargout, out, nargin-1, in+1);
      break;
    case 48:
      MyVector12_constructor_48(nargout, out, nargin-1, in+1);
      break;
    case 49:
      MyVector12_deconstructor_49(nargout, out, nargin-1, in+1);
      break;
    case 50:
      MultipleTemplatesIntDouble_collectorInsertAndMakeBase_50(nargout, out, nargin-1, in+1);
      break;
    case 51:
      MultipleTemplatesIntDouble_deconstructor_51(nargout, out, nargin-1, in+1);
      break;
    case 52:
      MultipleTemplatesIntFloat_collectorInsertAndMakeBase_52(nargout, out, nargin-1, in+1);
      break;
    case 53:
      MultipleTemplatesIntFloat_deconstructor_53(nargout, out, nargin-1, in+1);
      break;
    case 54:
      ForwardKinematics_collectorInsertAndMakeBase_54(nargout, out, nargin-1, in+1);
      break;
    case 55:
      ForwardKinematics_constructor_55(nargout, out, nargin-1, in+1);
      break;
    case 56:
      ForwardKinematics_deconstructor_56(nargout, out, nargin-1, in+1);
      break;
    case 57:
      TemplatedConstructor_collectorInsertAndMakeBase_57(nargout, out, nargin-1, in+1);
      break;
    case 58:
      TemplatedConstructor_constructor_58(nargout, out, nargin-1, in+1);
      break;
    case 59:
      TemplatedConstructor_constructor_59(nargout, out, nargin-1, in+1);
      break;
    case 60:
      TemplatedConstructor_constructor_60(nargout, out, nargin-1, in+1);
      break;
    case 61:
      TemplatedConstructor_constructor_61(nargout, out, nargin-1, in+1);
      break;
    case 62:
      TemplatedConstructor_deconstructor_62(nargout, out, nargin-1, in+1);
      break;
    case 63:
      MyFactorPosePoint2_collectorInsertAndMakeBase_63(nargout, out, nargin-1, in+1);
      break;
    case 64:
      MyFactorPosePoint2_constructor_64(nargout, out, nargin-1, in+1);
      break;
    case 65:
      MyFactorPosePoint2_deconstructor_65(nargout, out, nargin-1, in+1);
      break;
    case 66:
      MyFactorPosePoint2_print_66(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

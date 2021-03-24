#include <gtwrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>

#include <folder/path/to/Test.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

typedef MyTemplate<gtsam::Point2> MyTemplatePoint2;
typedef MyTemplate<gtsam::Matrix> MyTemplateMatrix;
typedef PrimitiveRef<double> PrimitiveRefDouble;
typedef MyVector<3> MyVector3;
typedef MyVector<12> MyVector12;
typedef MultipleTemplates<int, double> MultipleTemplatesIntDouble;
typedef MultipleTemplates<int, float> MultipleTemplatesIntFloat;
typedef MyFactor<gtsam::Pose2, gtsam::Matrix> MyFactorPosePoint2;

BOOST_CLASS_EXPORT_GUID(gtsam::Point2, "gtsamPoint2");
BOOST_CLASS_EXPORT_GUID(gtsam::Point3, "gtsamPoint3");

typedef std::set<boost::shared_ptr<gtsam::Point2>*> Collector_gtsamPoint2;
static Collector_gtsamPoint2 collector_gtsamPoint2;
typedef std::set<boost::shared_ptr<gtsam::Point3>*> Collector_gtsamPoint3;
static Collector_gtsamPoint3 collector_gtsamPoint3;
typedef std::set<boost::shared_ptr<Test>*> Collector_Test;
static Collector_Test collector_Test;
typedef std::set<boost::shared_ptr<MyBase>*> Collector_MyBase;
static Collector_MyBase collector_MyBase;
typedef std::set<boost::shared_ptr<MyTemplatePoint2>*> Collector_MyTemplatePoint2;
static Collector_MyTemplatePoint2 collector_MyTemplatePoint2;
typedef std::set<boost::shared_ptr<MyTemplateMatrix>*> Collector_MyTemplateMatrix;
static Collector_MyTemplateMatrix collector_MyTemplateMatrix;
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
typedef std::set<boost::shared_ptr<MyFactorPosePoint2>*> Collector_MyFactorPosePoint2;
static Collector_MyFactorPosePoint2 collector_MyFactorPosePoint2;

void _deleteAllObjects()
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  bool anyDeleted = false;
  { for(Collector_gtsamPoint2::iterator iter = collector_gtsamPoint2.begin();
      iter != collector_gtsamPoint2.end(); ) {
    delete *iter;
    collector_gtsamPoint2.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gtsamPoint3::iterator iter = collector_gtsamPoint3.begin();
      iter != collector_gtsamPoint3.end(); ) {
    delete *iter;
    collector_gtsamPoint3.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_Test::iterator iter = collector_Test.begin();
      iter != collector_Test.end(); ) {
    delete *iter;
    collector_Test.erase(iter++);
    anyDeleted = true;
  } }
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

void _geometry_RTTIRegister() {
  const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_geometry_rttiRegistry_created");
  if(!alreadyCreated) {
    std::map<std::string, std::string> types;
    types.insert(std::make_pair(typeid(MyBase).name(), "MyBase"));
    types.insert(std::make_pair(typeid(MyTemplatePoint2).name(), "MyTemplatePoint2"));
    types.insert(std::make_pair(typeid(MyTemplateMatrix).name(), "MyTemplateMatrix"));

    mxArray *registry = mexGetVariable("global", "gtsamwrap_rttiRegistry");
    if(!registry)
      registry = mxCreateStructMatrix(1, 1, 0, NULL);
    typedef std::pair<std::string, std::string> StringPair;
    for(const StringPair& rtti_matlab: types) {
      int fieldId = mxAddField(registry, rtti_matlab.first.c_str());
      if(fieldId < 0)
        mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
      mxArray *matlabName = mxCreateString(rtti_matlab.second.c_str());
      mxSetFieldByNumber(registry, 0, fieldId, matlabName);
    }
    if(mexPutVariable("global", "gtsamwrap_rttiRegistry", registry) != 0)
      mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
    mxDestroyArray(registry);
    
    mxArray *newAlreadyCreated = mxCreateNumericMatrix(0, 0, mxINT8_CLASS, mxREAL);
    if(mexPutVariable("global", "gtsam_geometry_rttiRegistry_created", newAlreadyCreated) != 0)
      mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
    mxDestroyArray(newAlreadyCreated);
  }
}

void gtsamPoint2_collectorInsertAndMakeBase_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Point2> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamPoint2.insert(self);
}

void gtsamPoint2_constructor_1(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Point2> Shared;

  Shared *self = new Shared(new gtsam::Point2());
  collector_gtsamPoint2.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gtsamPoint2_constructor_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Point2> Shared;

  double x = unwrap< double >(in[0]);
  double y = unwrap< double >(in[1]);
  Shared *self = new Shared(new gtsam::Point2(x,y));
  collector_gtsamPoint2.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gtsamPoint2_deconstructor_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Point2> Shared;
  checkArguments("delete_gtsamPoint2",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamPoint2::iterator item;
  item = collector_gtsamPoint2.find(self);
  if(item != collector_gtsamPoint2.end()) {
    delete self;
    collector_gtsamPoint2.erase(item);
  }
}

void gtsamPoint2_argChar_4(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("argChar",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  char a = unwrap< char >(in[1]);
  obj->argChar(a);
}

void gtsamPoint2_argChar_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("argChar",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  boost::shared_ptr<char> a = unwrap_shared_ptr< char >(in[1], "ptr_char");
  obj->argChar(a);
}

void gtsamPoint2_argChar_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("argChar",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  char a = unwrap< char >(in[1]);
  obj->argChar(a);
}

void gtsamPoint2_argChar_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("argChar",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  boost::shared_ptr<char> a = unwrap_shared_ptr< char >(in[1], "ptr_char");
  obj->argChar(a);
}

void gtsamPoint2_argChar_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("argChar",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  boost::shared_ptr<char> a = unwrap_shared_ptr< char >(in[1], "ptr_char");
  obj->argChar(a);
}

void gtsamPoint2_argChar_9(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("argChar",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  char a = unwrap< char >(in[1]);
  obj->argChar(a);
}

void gtsamPoint2_argChar_10(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("argChar",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  boost::shared_ptr<char> a = unwrap_shared_ptr< char >(in[1], "ptr_char");
  obj->argChar(a);
}

void gtsamPoint2_argUChar_11(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("argUChar",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  unsigned char a = unwrap< unsigned char >(in[1]);
  obj->argUChar(a);
}

void gtsamPoint2_dim_12(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("dim",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap< int >(obj->dim());
}

void gtsamPoint2_eigenArguments_13(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("eigenArguments",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  Vector v = unwrap< Vector >(in[1]);
  Matrix m = unwrap< Matrix >(in[2]);
  obj->eigenArguments(v,m);
}

void gtsamPoint2_returnChar_14(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("returnChar",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap< char >(obj->returnChar());
}

void gtsamPoint2_vectorConfusion_15(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("vectorConfusion",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap_shared_ptr(boost::make_shared<VectorNotEigen>(obj->vectorConfusion()),"VectorNotEigen", false);
}

void gtsamPoint2_x_16(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("x",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap< double >(obj->x());
}

void gtsamPoint2_y_17(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("y",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap< double >(obj->y());
}

void gtsamPoint3_collectorInsertAndMakeBase_18(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Point3> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamPoint3.insert(self);
}

void gtsamPoint3_constructor_19(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Point3> Shared;

  double x = unwrap< double >(in[0]);
  double y = unwrap< double >(in[1]);
  double z = unwrap< double >(in[2]);
  Shared *self = new Shared(new gtsam::Point3(x,y,z));
  collector_gtsamPoint3.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gtsamPoint3_deconstructor_20(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Point3> Shared;
  checkArguments("delete_gtsamPoint3",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamPoint3::iterator item;
  item = collector_gtsamPoint3.find(self);
  if(item != collector_gtsamPoint3.end()) {
    delete self;
    collector_gtsamPoint3.erase(item);
  }
}

void gtsamPoint3_norm_21(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("norm",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point3>(in[0], "ptr_gtsamPoint3");
  out[0] = wrap< double >(obj->norm());
}

void gtsamPoint3_string_serialize_22(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Point3> Shared;
  checkArguments("string_serialize",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gtsam::Point3>(in[0], "ptr_gtsamPoint3");
  ostringstream out_archive_stream;
  boost::archive::text_oarchive out_archive(out_archive_stream);
  out_archive << *obj;
  out[0] = wrap< string >(out_archive_stream.str());
}
void gtsamPoint3_StaticFunctionRet_23(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gtsamPoint3.StaticFunctionRet",nargout,nargin,1);
  double z = unwrap< double >(in[0]);
  out[0] = wrap< Point3 >(gtsam::Point3::StaticFunctionRet(z));
}

void gtsamPoint3_staticFunction_24(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gtsamPoint3.staticFunction",nargout,nargin,0);
  out[0] = wrap< double >(gtsam::Point3::staticFunction());
}

void gtsamPoint3_string_deserialize_25(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Point3> Shared;
  checkArguments("gtsamPoint3.string_deserialize",nargout,nargin,1);
  string serialized = unwrap< string >(in[0]);
  istringstream in_archive_stream(serialized);
  boost::archive::text_iarchive in_archive(in_archive_stream);
  Shared output(new gtsam::Point3());
  in_archive >> *output;
  out[0] = wrap_shared_ptr(output,"gtsam.Point3", false);
}
void Test_collectorInsertAndMakeBase_26(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<Test> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_Test.insert(self);
}

void Test_constructor_27(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<Test> Shared;

  Shared *self = new Shared(new Test());
  collector_Test.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void Test_constructor_28(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void Test_deconstructor_29(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void Test_arg_EigenConstRef_30(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("arg_EigenConstRef",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Matrix value = unwrap< Matrix >(in[1]);
  obj->arg_EigenConstRef(value);
}

void Test_create_MixedPtrs_31(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_MixedPtrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  auto pairResult = obj->create_MixedPtrs();
  out[0] = wrap_shared_ptr(boost::make_shared<Test>(pairResult.first),"Test", false);
  out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
}

void Test_create_ptrs_32(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_ptrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  auto pairResult = obj->create_ptrs();
  out[0] = wrap_shared_ptr(pairResult.first,"Test", false);
  out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
}

void Test_print_33(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("print",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  obj->print();
}

void Test_return_Point2Ptr_34(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Point2Ptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  bool value = unwrap< bool >(in[1]);
  {
  boost::shared_ptr<Point2> shared(obj->return_Point2Ptr(value));
  out[0] = wrap_shared_ptr(shared,"Point2");
  }
}

void Test_return_Test_35(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Test",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<Test> value = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  out[0] = wrap_shared_ptr(boost::make_shared<Test>(obj->return_Test(value)),"Test", false);
}

void Test_return_TestPtr_36(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_TestPtr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<Test> value = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  out[0] = wrap_shared_ptr(obj->return_TestPtr(value),"Test", false);
}

void Test_return_bool_37(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_bool",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  bool value = unwrap< bool >(in[1]);
  out[0] = wrap< bool >(obj->return_bool(value));
}

void Test_return_double_38(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_double",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  double value = unwrap< double >(in[1]);
  out[0] = wrap< double >(obj->return_double(value));
}

void Test_return_field_39(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_field",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Test& t = *unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  out[0] = wrap< bool >(obj->return_field(t));
}

void Test_return_int_40(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_int",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  int value = unwrap< int >(in[1]);
  out[0] = wrap< int >(obj->return_int(value));
}

void Test_return_matrix1_41(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_matrix1",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Matrix value = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->return_matrix1(value));
}

void Test_return_matrix2_42(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_matrix2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Matrix value = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->return_matrix2(value));
}

void Test_return_pair_43(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_pair",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector v = unwrap< Vector >(in[1]);
  Matrix A = unwrap< Matrix >(in[2]);
  auto pairResult = obj->return_pair(v,A);
  out[0] = wrap< Vector >(pairResult.first);
  out[1] = wrap< Matrix >(pairResult.second);
}

void Test_return_pair_44(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_pair",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector v = unwrap< Vector >(in[1]);
  auto pairResult = obj->return_pair(v);
  out[0] = wrap< Vector >(pairResult.first);
  out[1] = wrap< Matrix >(pairResult.second);
}

void Test_return_ptrs_45(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_ptrs",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<Test> p1 = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  boost::shared_ptr<Test> p2 = unwrap_shared_ptr< Test >(in[2], "ptr_Test");
  auto pairResult = obj->return_ptrs(p1,p2);
  out[0] = wrap_shared_ptr(pairResult.first,"Test", false);
  out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
}

void Test_return_size_t_46(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_size_t",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  size_t value = unwrap< size_t >(in[1]);
  out[0] = wrap< size_t >(obj->return_size_t(value));
}

void Test_return_string_47(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_string",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  string value = unwrap< string >(in[1]);
  out[0] = wrap< string >(obj->return_string(value));
}

void Test_return_vector1_48(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_vector1",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector value = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->return_vector1(value));
}

void Test_return_vector2_49(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_vector2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector value = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->return_vector2(value));
}

void MyBase_collectorInsertAndMakeBase_50(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyBase> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyBase.insert(self);
}

void MyBase_upcastFromVoid_51(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyBase> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<MyBase>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void MyBase_deconstructor_52(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_collectorInsertAndMakeBase_53(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Point2>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyTemplatePoint2.insert(self);

  typedef boost::shared_ptr<MyBase> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void MyTemplatePoint2_upcastFromVoid_54(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Point2>> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<MyTemplate<gtsam::Point2>>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void MyTemplatePoint2_constructor_55(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_deconstructor_56(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_accept_T_57(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  obj->accept_T(value);
}

void MyTemplatePoint2_accept_Tptr_58(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  obj->accept_Tptr(value);
}

void MyTemplatePoint2_create_MixedPtrs_59(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_create_ptrs_60(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_return_T_61(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  out[0] = wrap< Point2 >(obj->return_T(value));
}

void MyTemplatePoint2_return_Tptr_62(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  {
  boost::shared_ptr<Point2> shared(obj->return_Tptr(value));
  out[0] = wrap_shared_ptr(shared,"Point2");
  }
}

void MyTemplatePoint2_return_ptrs_63(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_templatedMethod_64(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodMatrix",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Matrix t = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->templatedMethod<gtsam::Matrix>(t));
}

void MyTemplatePoint2_templatedMethod_65(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 t = unwrap< Point2 >(in[1]);
  out[0] = wrap< Point2 >(obj->templatedMethod<gtsam::Point2>(t));
}

void MyTemplatePoint2_templatedMethod_66(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint3",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point3 t = unwrap< Point3 >(in[1]);
  out[0] = wrap< Point3 >(obj->templatedMethod<gtsam::Point3>(t));
}

void MyTemplatePoint2_templatedMethod_67(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodVector",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Vector t = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->templatedMethod<gtsam::Vector>(t));
}

void MyTemplatePoint2_Level_68(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MyTemplatePoint2.Level",nargout,nargin,1);
  Point2 K = unwrap< Point2 >(in[0]);
  out[0] = wrap_shared_ptr(boost::make_shared<MyTemplate<Point2>>(MyTemplate<gtsam::Point2>::Level(K)),"MyTemplatePoint2", false);
}

void MyTemplateMatrix_collectorInsertAndMakeBase_69(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Matrix>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyTemplateMatrix.insert(self);

  typedef boost::shared_ptr<MyBase> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void MyTemplateMatrix_upcastFromVoid_70(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Matrix>> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<MyTemplate<gtsam::Matrix>>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void MyTemplateMatrix_constructor_71(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_deconstructor_72(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_accept_T_73(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  obj->accept_T(value);
}

void MyTemplateMatrix_accept_Tptr_74(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  obj->accept_Tptr(value);
}

void MyTemplateMatrix_create_MixedPtrs_75(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_create_ptrs_76(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_return_T_77(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->return_T(value));
}

void MyTemplateMatrix_return_Tptr_78(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  {
  boost::shared_ptr<Matrix> shared(obj->return_Tptr(value));
  out[0] = wrap_shared_ptr(shared,"Matrix");
  }
}

void MyTemplateMatrix_return_ptrs_79(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_templatedMethod_80(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodMatrix",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix t = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->templatedMethod<gtsam::Matrix>(t));
}

void MyTemplateMatrix_templatedMethod_81(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Point2 t = unwrap< Point2 >(in[1]);
  out[0] = wrap< Point2 >(obj->templatedMethod<gtsam::Point2>(t));
}

void MyTemplateMatrix_templatedMethod_82(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint3",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Point3 t = unwrap< Point3 >(in[1]);
  out[0] = wrap< Point3 >(obj->templatedMethod<gtsam::Point3>(t));
}

void MyTemplateMatrix_templatedMethod_83(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodVector",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Vector t = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->templatedMethod<gtsam::Vector>(t));
}

void MyTemplateMatrix_Level_84(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MyTemplateMatrix.Level",nargout,nargin,1);
  Matrix K = unwrap< Matrix >(in[0]);
  out[0] = wrap_shared_ptr(boost::make_shared<MyTemplate<Matrix>>(MyTemplate<gtsam::Matrix>::Level(K)),"MyTemplateMatrix", false);
}

void PrimitiveRefDouble_collectorInsertAndMakeBase_85(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<PrimitiveRef<double>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_PrimitiveRefDouble.insert(self);
}

void PrimitiveRefDouble_constructor_86(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<PrimitiveRef<double>> Shared;

  Shared *self = new Shared(new PrimitiveRef<double>());
  collector_PrimitiveRefDouble.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void PrimitiveRefDouble_deconstructor_87(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void PrimitiveRefDouble_Brutal_88(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("PrimitiveRefDouble.Brutal",nargout,nargin,1);
  double t = unwrap< double >(in[0]);
  out[0] = wrap_shared_ptr(boost::make_shared<PrimitiveRef<double>>(PrimitiveRef<double>::Brutal(t)),"PrimitiveRefdouble", false);
}

void MyVector3_collectorInsertAndMakeBase_89(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<3>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyVector3.insert(self);
}

void MyVector3_constructor_90(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<3>> Shared;

  Shared *self = new Shared(new MyVector<3>());
  collector_MyVector3.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void MyVector3_deconstructor_91(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyVector12_collectorInsertAndMakeBase_92(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<12>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyVector12.insert(self);
}

void MyVector12_constructor_93(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<12>> Shared;

  Shared *self = new Shared(new MyVector<12>());
  collector_MyVector12.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void MyVector12_deconstructor_94(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MultipleTemplatesIntDouble_collectorInsertAndMakeBase_95(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MultipleTemplates<int, double>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MultipleTemplatesIntDouble.insert(self);
}

void MultipleTemplatesIntDouble_deconstructor_96(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MultipleTemplatesIntFloat_collectorInsertAndMakeBase_97(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MultipleTemplates<int, float>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MultipleTemplatesIntFloat.insert(self);
}

void MultipleTemplatesIntFloat_deconstructor_98(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyFactorPosePoint2_collectorInsertAndMakeBase_99(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyFactorPosePoint2.insert(self);
}

void MyFactorPosePoint2_constructor_100(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyFactorPosePoint2_deconstructor_101(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void load2D_102(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("load2D",nargout,nargin,5);
  string filename = unwrap< string >(in[0]);
  boost::shared_ptr<Test> model = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  int maxID = unwrap< int >(in[2]);
  bool addNoise = unwrap< bool >(in[3]);
  bool smart = unwrap< bool >(in[4]);
  auto pairResult = load2D(filename,model,maxID,addNoise,smart);
  out[0] = wrap_shared_ptr(pairResult.first,"gtsam.NonlinearFactorGraph", false);
  out[1] = wrap_shared_ptr(pairResult.second,"gtsam.Values", false);
}
void load2D_103(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("load2D",nargout,nargin,5);
  string filename = unwrap< string >(in[0]);
  boost::shared_ptr<gtsam::noiseModel::Diagonal> model = unwrap_shared_ptr< gtsam::noiseModel::Diagonal >(in[1], "ptr_gtsamnoiseModelDiagonal");
  int maxID = unwrap< int >(in[2]);
  bool addNoise = unwrap< bool >(in[3]);
  bool smart = unwrap< bool >(in[4]);
  auto pairResult = load2D(filename,model,maxID,addNoise,smart);
  out[0] = wrap_shared_ptr(pairResult.first,"gtsam.NonlinearFactorGraph", false);
  out[1] = wrap_shared_ptr(pairResult.second,"gtsam.Values", false);
}
void load2D_104(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("load2D",nargout,nargin,2);
  string filename = unwrap< string >(in[0]);
  boost::shared_ptr<gtsam::noiseModel::Diagonal> model = unwrap_shared_ptr< gtsam::noiseModel::Diagonal >(in[1], "ptr_gtsamnoiseModelDiagonal");
  auto pairResult = load2D(filename,model);
  out[0] = wrap_shared_ptr(pairResult.first,"gtsam.NonlinearFactorGraph", false);
  out[1] = wrap_shared_ptr(pairResult.second,"gtsam.Values", false);
}
void aGlobalFunction_105(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("aGlobalFunction",nargout,nargin,0);
  out[0] = wrap< Vector >(aGlobalFunction());
}
void overloadedGlobalFunction_106(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("overloadedGlobalFunction",nargout,nargin,1);
  int a = unwrap< int >(in[0]);
  out[0] = wrap< Vector >(overloadedGlobalFunction(a));
}
void overloadedGlobalFunction_107(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("overloadedGlobalFunction",nargout,nargin,2);
  int a = unwrap< int >(in[0]);
  double b = unwrap< double >(in[1]);
  out[0] = wrap< Vector >(overloadedGlobalFunction(a,b));
}
void MultiTemplatedFunctionStringSize_tDouble_108(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MultiTemplatedFunctionStringSize_tDouble",nargout,nargin,2);
  T& x = *unwrap_shared_ptr< T >(in[0], "ptr_T");
  size_t y = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(MultiTemplatedFunctionStringSize_tDouble(x,y));
}
void MultiTemplatedFunctionDoubleSize_tDouble_109(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MultiTemplatedFunctionDoubleSize_tDouble",nargout,nargin,2);
  T& x = *unwrap_shared_ptr< T >(in[0], "ptr_T");
  size_t y = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(MultiTemplatedFunctionDoubleSize_tDouble(x,y));
}
void TemplatedFunctionRot3_110(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("TemplatedFunctionRot3",nargout,nargin,1);
  gtsam::Rot3& t = *unwrap_shared_ptr< gtsam::Rot3 >(in[0], "ptr_gtsamRot3");
  TemplatedFunctionRot3(t);
}

void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  _geometry_RTTIRegister();

  int id = unwrap<int>(in[0]);

  try {
    switch(id) {
    case 0:
      gtsamPoint2_collectorInsertAndMakeBase_0(nargout, out, nargin-1, in+1);
      break;
    case 1:
      gtsamPoint2_constructor_1(nargout, out, nargin-1, in+1);
      break;
    case 2:
      gtsamPoint2_constructor_2(nargout, out, nargin-1, in+1);
      break;
    case 3:
      gtsamPoint2_deconstructor_3(nargout, out, nargin-1, in+1);
      break;
    case 4:
      gtsamPoint2_argChar_4(nargout, out, nargin-1, in+1);
      break;
    case 5:
      gtsamPoint2_argChar_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      gtsamPoint2_argChar_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      gtsamPoint2_argChar_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      gtsamPoint2_argChar_8(nargout, out, nargin-1, in+1);
      break;
    case 9:
      gtsamPoint2_argChar_9(nargout, out, nargin-1, in+1);
      break;
    case 10:
      gtsamPoint2_argChar_10(nargout, out, nargin-1, in+1);
      break;
    case 11:
      gtsamPoint2_argUChar_11(nargout, out, nargin-1, in+1);
      break;
    case 12:
      gtsamPoint2_dim_12(nargout, out, nargin-1, in+1);
      break;
    case 13:
      gtsamPoint2_eigenArguments_13(nargout, out, nargin-1, in+1);
      break;
    case 14:
      gtsamPoint2_returnChar_14(nargout, out, nargin-1, in+1);
      break;
    case 15:
      gtsamPoint2_vectorConfusion_15(nargout, out, nargin-1, in+1);
      break;
    case 16:
      gtsamPoint2_x_16(nargout, out, nargin-1, in+1);
      break;
    case 17:
      gtsamPoint2_y_17(nargout, out, nargin-1, in+1);
      break;
    case 18:
      gtsamPoint3_collectorInsertAndMakeBase_18(nargout, out, nargin-1, in+1);
      break;
    case 19:
      gtsamPoint3_constructor_19(nargout, out, nargin-1, in+1);
      break;
    case 20:
      gtsamPoint3_deconstructor_20(nargout, out, nargin-1, in+1);
      break;
    case 21:
      gtsamPoint3_norm_21(nargout, out, nargin-1, in+1);
      break;
    case 22:
      gtsamPoint3_string_serialize_22(nargout, out, nargin-1, in+1);
      break;
    case 23:
      gtsamPoint3_StaticFunctionRet_23(nargout, out, nargin-1, in+1);
      break;
    case 24:
      gtsamPoint3_staticFunction_24(nargout, out, nargin-1, in+1);
      break;
    case 25:
      gtsamPoint3_string_deserialize_25(nargout, out, nargin-1, in+1);
      break;
    case 26:
      Test_collectorInsertAndMakeBase_26(nargout, out, nargin-1, in+1);
      break;
    case 27:
      Test_constructor_27(nargout, out, nargin-1, in+1);
      break;
    case 28:
      Test_constructor_28(nargout, out, nargin-1, in+1);
      break;
    case 29:
      Test_deconstructor_29(nargout, out, nargin-1, in+1);
      break;
    case 30:
      Test_arg_EigenConstRef_30(nargout, out, nargin-1, in+1);
      break;
    case 31:
      Test_create_MixedPtrs_31(nargout, out, nargin-1, in+1);
      break;
    case 32:
      Test_create_ptrs_32(nargout, out, nargin-1, in+1);
      break;
    case 33:
      Test_print_33(nargout, out, nargin-1, in+1);
      break;
    case 34:
      Test_return_Point2Ptr_34(nargout, out, nargin-1, in+1);
      break;
    case 35:
      Test_return_Test_35(nargout, out, nargin-1, in+1);
      break;
    case 36:
      Test_return_TestPtr_36(nargout, out, nargin-1, in+1);
      break;
    case 37:
      Test_return_bool_37(nargout, out, nargin-1, in+1);
      break;
    case 38:
      Test_return_double_38(nargout, out, nargin-1, in+1);
      break;
    case 39:
      Test_return_field_39(nargout, out, nargin-1, in+1);
      break;
    case 40:
      Test_return_int_40(nargout, out, nargin-1, in+1);
      break;
    case 41:
      Test_return_matrix1_41(nargout, out, nargin-1, in+1);
      break;
    case 42:
      Test_return_matrix2_42(nargout, out, nargin-1, in+1);
      break;
    case 43:
      Test_return_pair_43(nargout, out, nargin-1, in+1);
      break;
    case 44:
      Test_return_pair_44(nargout, out, nargin-1, in+1);
      break;
    case 45:
      Test_return_ptrs_45(nargout, out, nargin-1, in+1);
      break;
    case 46:
      Test_return_size_t_46(nargout, out, nargin-1, in+1);
      break;
    case 47:
      Test_return_string_47(nargout, out, nargin-1, in+1);
      break;
    case 48:
      Test_return_vector1_48(nargout, out, nargin-1, in+1);
      break;
    case 49:
      Test_return_vector2_49(nargout, out, nargin-1, in+1);
      break;
    case 50:
      MyBase_collectorInsertAndMakeBase_50(nargout, out, nargin-1, in+1);
      break;
    case 51:
      MyBase_upcastFromVoid_51(nargout, out, nargin-1, in+1);
      break;
    case 52:
      MyBase_deconstructor_52(nargout, out, nargin-1, in+1);
      break;
    case 53:
      MyTemplatePoint2_collectorInsertAndMakeBase_53(nargout, out, nargin-1, in+1);
      break;
    case 54:
      MyTemplatePoint2_upcastFromVoid_54(nargout, out, nargin-1, in+1);
      break;
    case 55:
      MyTemplatePoint2_constructor_55(nargout, out, nargin-1, in+1);
      break;
    case 56:
      MyTemplatePoint2_deconstructor_56(nargout, out, nargin-1, in+1);
      break;
    case 57:
      MyTemplatePoint2_accept_T_57(nargout, out, nargin-1, in+1);
      break;
    case 58:
      MyTemplatePoint2_accept_Tptr_58(nargout, out, nargin-1, in+1);
      break;
    case 59:
      MyTemplatePoint2_create_MixedPtrs_59(nargout, out, nargin-1, in+1);
      break;
    case 60:
      MyTemplatePoint2_create_ptrs_60(nargout, out, nargin-1, in+1);
      break;
    case 61:
      MyTemplatePoint2_return_T_61(nargout, out, nargin-1, in+1);
      break;
    case 62:
      MyTemplatePoint2_return_Tptr_62(nargout, out, nargin-1, in+1);
      break;
    case 63:
      MyTemplatePoint2_return_ptrs_63(nargout, out, nargin-1, in+1);
      break;
    case 64:
      MyTemplatePoint2_templatedMethod_64(nargout, out, nargin-1, in+1);
      break;
    case 65:
      MyTemplatePoint2_templatedMethod_65(nargout, out, nargin-1, in+1);
      break;
    case 66:
      MyTemplatePoint2_templatedMethod_66(nargout, out, nargin-1, in+1);
      break;
    case 67:
      MyTemplatePoint2_templatedMethod_67(nargout, out, nargin-1, in+1);
      break;
    case 68:
      MyTemplatePoint2_Level_68(nargout, out, nargin-1, in+1);
      break;
    case 69:
      MyTemplateMatrix_collectorInsertAndMakeBase_69(nargout, out, nargin-1, in+1);
      break;
    case 70:
      MyTemplateMatrix_upcastFromVoid_70(nargout, out, nargin-1, in+1);
      break;
    case 71:
      MyTemplateMatrix_constructor_71(nargout, out, nargin-1, in+1);
      break;
    case 72:
      MyTemplateMatrix_deconstructor_72(nargout, out, nargin-1, in+1);
      break;
    case 73:
      MyTemplateMatrix_accept_T_73(nargout, out, nargin-1, in+1);
      break;
    case 74:
      MyTemplateMatrix_accept_Tptr_74(nargout, out, nargin-1, in+1);
      break;
    case 75:
      MyTemplateMatrix_create_MixedPtrs_75(nargout, out, nargin-1, in+1);
      break;
    case 76:
      MyTemplateMatrix_create_ptrs_76(nargout, out, nargin-1, in+1);
      break;
    case 77:
      MyTemplateMatrix_return_T_77(nargout, out, nargin-1, in+1);
      break;
    case 78:
      MyTemplateMatrix_return_Tptr_78(nargout, out, nargin-1, in+1);
      break;
    case 79:
      MyTemplateMatrix_return_ptrs_79(nargout, out, nargin-1, in+1);
      break;
    case 80:
      MyTemplateMatrix_templatedMethod_80(nargout, out, nargin-1, in+1);
      break;
    case 81:
      MyTemplateMatrix_templatedMethod_81(nargout, out, nargin-1, in+1);
      break;
    case 82:
      MyTemplateMatrix_templatedMethod_82(nargout, out, nargin-1, in+1);
      break;
    case 83:
      MyTemplateMatrix_templatedMethod_83(nargout, out, nargin-1, in+1);
      break;
    case 84:
      MyTemplateMatrix_Level_84(nargout, out, nargin-1, in+1);
      break;
    case 85:
      PrimitiveRefDouble_collectorInsertAndMakeBase_85(nargout, out, nargin-1, in+1);
      break;
    case 86:
      PrimitiveRefDouble_constructor_86(nargout, out, nargin-1, in+1);
      break;
    case 87:
      PrimitiveRefDouble_deconstructor_87(nargout, out, nargin-1, in+1);
      break;
    case 88:
      PrimitiveRefDouble_Brutal_88(nargout, out, nargin-1, in+1);
      break;
    case 89:
      MyVector3_collectorInsertAndMakeBase_89(nargout, out, nargin-1, in+1);
      break;
    case 90:
      MyVector3_constructor_90(nargout, out, nargin-1, in+1);
      break;
    case 91:
      MyVector3_deconstructor_91(nargout, out, nargin-1, in+1);
      break;
    case 92:
      MyVector12_collectorInsertAndMakeBase_92(nargout, out, nargin-1, in+1);
      break;
    case 93:
      MyVector12_constructor_93(nargout, out, nargin-1, in+1);
      break;
    case 94:
      MyVector12_deconstructor_94(nargout, out, nargin-1, in+1);
      break;
    case 95:
      MultipleTemplatesIntDouble_collectorInsertAndMakeBase_95(nargout, out, nargin-1, in+1);
      break;
    case 96:
      MultipleTemplatesIntDouble_deconstructor_96(nargout, out, nargin-1, in+1);
      break;
    case 97:
      MultipleTemplatesIntFloat_collectorInsertAndMakeBase_97(nargout, out, nargin-1, in+1);
      break;
    case 98:
      MultipleTemplatesIntFloat_deconstructor_98(nargout, out, nargin-1, in+1);
      break;
    case 99:
      MyFactorPosePoint2_collectorInsertAndMakeBase_99(nargout, out, nargin-1, in+1);
      break;
    case 100:
      MyFactorPosePoint2_constructor_100(nargout, out, nargin-1, in+1);
      break;
    case 101:
      MyFactorPosePoint2_deconstructor_101(nargout, out, nargin-1, in+1);
      break;
    case 102:
      load2D_102(nargout, out, nargin-1, in+1);
      break;
    case 103:
      load2D_103(nargout, out, nargin-1, in+1);
      break;
    case 104:
      load2D_104(nargout, out, nargin-1, in+1);
      break;
    case 105:
      aGlobalFunction_105(nargout, out, nargin-1, in+1);
      break;
    case 106:
      overloadedGlobalFunction_106(nargout, out, nargin-1, in+1);
      break;
    case 107:
      overloadedGlobalFunction_107(nargout, out, nargin-1, in+1);
      break;
    case 108:
      MultiTemplatedFunctionStringSize_tDouble_108(nargout, out, nargin-1, in+1);
      break;
    case 109:
      MultiTemplatedFunctionDoubleSize_tDouble_109(nargout, out, nargin-1, in+1);
      break;
    case 110:
      TemplatedFunctionRot3_110(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

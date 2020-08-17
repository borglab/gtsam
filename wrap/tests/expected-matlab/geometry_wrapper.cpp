#include <wrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>

#include <folder/path/to/Test.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

typedef MyTemplate<gtsam::Point2> MyTemplatePoint2;
typedef MyTemplate<gtsam::Matrix> MyTemplateMatrix;
typedef PrimitiveRef<double> PrimitiveRefdouble;
typedef MyVector<3> MyVector3;
typedef MyVector<12> MyVector12;
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
typedef std::set<boost::shared_ptr<PrimitiveRefdouble>*> Collector_PrimitiveRefdouble;
static Collector_PrimitiveRefdouble collector_PrimitiveRefdouble;
typedef std::set<boost::shared_ptr<MyVector3>*> Collector_MyVector3;
static Collector_MyVector3 collector_MyVector3;
typedef std::set<boost::shared_ptr<MyVector12>*> Collector_MyVector12;
static Collector_MyVector12 collector_MyVector12;
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
  { for(Collector_PrimitiveRefdouble::iterator iter = collector_PrimitiveRefdouble.begin();
      iter != collector_PrimitiveRefdouble.end(); ) {
    delete *iter;
    collector_PrimitiveRefdouble.erase(iter++);
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

void gtsamPoint2_argUChar_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("argUChar",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  unsigned char a = unwrap< unsigned char >(in[1]);
  obj->argUChar(a);
}

void gtsamPoint2_dim_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("dim",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap< int >(obj->dim());
}

void gtsamPoint2_eigenArguments_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("eigenArguments",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  Vector v = unwrap< Vector >(in[1]);
  Matrix m = unwrap< Matrix >(in[2]);
  obj->eigenArguments(v,m);
}

void gtsamPoint2_returnChar_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("returnChar",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap< char >(obj->returnChar());
}

void gtsamPoint2_vectorConfusion_9(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("vectorConfusion",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap_shared_ptr(boost::make_shared<VectorNotEigen>(obj->vectorConfusion()),"VectorNotEigen", false);
}

void gtsamPoint2_x_10(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("x",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap< double >(obj->x());
}

void gtsamPoint2_y_11(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("y",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point2>(in[0], "ptr_gtsamPoint2");
  out[0] = wrap< double >(obj->y());
}

void gtsamPoint3_collectorInsertAndMakeBase_12(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Point3> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamPoint3.insert(self);
}

void gtsamPoint3_constructor_13(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void gtsamPoint3_deconstructor_14(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void gtsamPoint3_norm_15(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("norm",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::Point3>(in[0], "ptr_gtsamPoint3");
  out[0] = wrap< double >(obj->norm());
}

void gtsamPoint3_string_serialize_16(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Point3> Shared;
  checkArguments("string_serialize",nargout,nargin-1,0);
  Shared obj = unwrap_shared_ptr<gtsam::Point3>(in[0], "ptr_gtsamPoint3");
  ostringstream out_archive_stream;
  boost::archive::text_oarchive out_archive(out_archive_stream);
  out_archive << *obj;
  out[0] = wrap< string >(out_archive_stream.str());
}
void gtsamPoint3_StaticFunctionRet_17(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gtsamPoint3.StaticFunctionRet",nargout,nargin,1);
  double z = unwrap< double >(in[0]);
  out[0] = wrap< Point3 >(gtsam::Point3::StaticFunctionRet(z));
}

void gtsamPoint3_staticFunction_18(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gtsamPoint3.staticFunction",nargout,nargin,0);
  out[0] = wrap< double >(gtsam::Point3::staticFunction());
}

void gtsamPoint3_string_deserialize_19(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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
void Test_collectorInsertAndMakeBase_20(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<Test> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_Test.insert(self);
}

void Test_constructor_21(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<Test> Shared;

  Shared *self = new Shared(new Test());
  collector_Test.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void Test_constructor_22(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void Test_deconstructor_23(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void Test_arg_EigenConstRef_24(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("arg_EigenConstRef",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Matrix value = unwrap< Matrix >(in[1]);
  obj->arg_EigenConstRef(value);
}

void Test_create_MixedPtrs_25(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_MixedPtrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  auto pairResult = obj->create_MixedPtrs();
  out[0] = wrap_shared_ptr(boost::make_shared<Test>(pairResult.first),"Test", false);
  out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
}

void Test_create_ptrs_26(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("create_ptrs",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  auto pairResult = obj->create_ptrs();
  out[0] = wrap_shared_ptr(pairResult.first,"Test", false);
  out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
}

void Test_print_27(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("print",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  obj->print();
}

void Test_return_Point2Ptr_28(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Point2Ptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  bool value = unwrap< bool >(in[1]);
  {
  boost::shared_ptr<Point2> shared(obj->return_Point2Ptr(value));
  out[0] = wrap_shared_ptr(shared,"Point2");
  }
}

void Test_return_Test_29(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Test",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<Test> value = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  out[0] = wrap_shared_ptr(boost::make_shared<Test>(obj->return_Test(value)),"Test", false);
}

void Test_return_TestPtr_30(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_TestPtr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<Test> value = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  out[0] = wrap_shared_ptr(obj->return_TestPtr(value),"Test", false);
}

void Test_return_bool_31(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_bool",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  bool value = unwrap< bool >(in[1]);
  out[0] = wrap< bool >(obj->return_bool(value));
}

void Test_return_double_32(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_double",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  double value = unwrap< double >(in[1]);
  out[0] = wrap< double >(obj->return_double(value));
}

void Test_return_field_33(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_field",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Test& t = *unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  out[0] = wrap< bool >(obj->return_field(t));
}

void Test_return_int_34(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_int",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  int value = unwrap< int >(in[1]);
  out[0] = wrap< int >(obj->return_int(value));
}

void Test_return_matrix1_35(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_matrix1",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Matrix value = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->return_matrix1(value));
}

void Test_return_matrix2_36(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_matrix2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Matrix value = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->return_matrix2(value));
}

void Test_return_pair_37(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_pair",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector v = unwrap< Vector >(in[1]);
  Matrix A = unwrap< Matrix >(in[2]);
  auto pairResult = obj->return_pair(v,A);
  out[0] = wrap< Vector >(pairResult.first);
  out[1] = wrap< Matrix >(pairResult.second);
}

void Test_return_pair_38(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_pair",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector v = unwrap< Vector >(in[1]);
  auto pairResult = obj->return_pair(v);
  out[0] = wrap< Vector >(pairResult.first);
  out[1] = wrap< Matrix >(pairResult.second);
}

void Test_return_ptrs_39(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_ptrs",nargout,nargin-1,2);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  boost::shared_ptr<Test> p1 = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
  boost::shared_ptr<Test> p2 = unwrap_shared_ptr< Test >(in[2], "ptr_Test");
  auto pairResult = obj->return_ptrs(p1,p2);
  out[0] = wrap_shared_ptr(pairResult.first,"Test", false);
  out[1] = wrap_shared_ptr(pairResult.second,"Test", false);
}

void Test_return_size_t_40(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_size_t",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  size_t value = unwrap< size_t >(in[1]);
  out[0] = wrap< size_t >(obj->return_size_t(value));
}

void Test_return_string_41(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_string",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  string value = unwrap< string >(in[1]);
  out[0] = wrap< string >(obj->return_string(value));
}

void Test_return_vector1_42(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_vector1",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector value = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->return_vector1(value));
}

void Test_return_vector2_43(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_vector2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Test>(in[0], "ptr_Test");
  Vector value = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->return_vector2(value));
}

void MyBase_collectorInsertAndMakeBase_44(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyBase> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyBase.insert(self);
}

void MyBase_upcastFromVoid_45(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyBase> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<MyBase>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void MyBase_deconstructor_46(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_collectorInsertAndMakeBase_47(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Point2>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyTemplatePoint2.insert(self);

  typedef boost::shared_ptr<MyBase> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void MyTemplatePoint2_upcastFromVoid_48(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Point2>> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<MyTemplate<gtsam::Point2>>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void MyTemplatePoint2_constructor_49(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_deconstructor_50(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_accept_T_51(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  obj->accept_T(value);
}

void MyTemplatePoint2_accept_Tptr_52(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  obj->accept_Tptr(value);
}

void MyTemplatePoint2_create_MixedPtrs_53(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_create_ptrs_54(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_return_T_55(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  out[0] = wrap< Point2 >(obj->return_T(value));
}

void MyTemplatePoint2_return_Tptr_56(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 value = unwrap< Point2 >(in[1]);
  {
  boost::shared_ptr<Point2> shared(obj->return_Tptr(value));
  out[0] = wrap_shared_ptr(shared,"Point2");
  }
}

void MyTemplatePoint2_return_ptrs_57(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplatePoint2_templatedMethod_58(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodMatrix",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Matrix t = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->templatedMethod<gtsam::Matrix>(t));
}

void MyTemplatePoint2_templatedMethod_59(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point2 t = unwrap< Point2 >(in[1]);
  out[0] = wrap< Point2 >(obj->templatedMethod<gtsam::Point2>(t));
}

void MyTemplatePoint2_templatedMethod_60(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint3",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Point3 t = unwrap< Point3 >(in[1]);
  out[0] = wrap< Point3 >(obj->templatedMethod<gtsam::Point3>(t));
}

void MyTemplatePoint2_templatedMethod_61(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodVector",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Point2>>(in[0], "ptr_MyTemplatePoint2");
  Vector t = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->templatedMethod<gtsam::Vector>(t));
}

void MyTemplatePoint2_Level_62(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MyTemplatePoint2.Level",nargout,nargin,1);
  Point2 K = unwrap< Point2 >(in[0]);
  out[0] = wrap_shared_ptr(boost::make_shared<MyTemplate<Point2>>(MyTemplate<gtsam::Point2>::Level(K)),"MyTemplatePoint2", false);
}

void MyTemplateMatrix_collectorInsertAndMakeBase_63(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Matrix>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyTemplateMatrix.insert(self);

  typedef boost::shared_ptr<MyBase> SharedBase;
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
}

void MyTemplateMatrix_upcastFromVoid_64(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyTemplate<gtsam::Matrix>> Shared;
  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  Shared *self = new Shared(boost::static_pointer_cast<MyTemplate<gtsam::Matrix>>(*asVoid));
  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
}

void MyTemplateMatrix_constructor_65(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_deconstructor_66(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_accept_T_67(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  obj->accept_T(value);
}

void MyTemplateMatrix_accept_Tptr_68(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("accept_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  obj->accept_Tptr(value);
}

void MyTemplateMatrix_create_MixedPtrs_69(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_create_ptrs_70(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_return_T_71(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_T",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->return_T(value));
}

void MyTemplateMatrix_return_Tptr_72(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("return_Tptr",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix value = unwrap< Matrix >(in[1]);
  {
  boost::shared_ptr<Matrix> shared(obj->return_Tptr(value));
  out[0] = wrap_shared_ptr(shared,"Matrix");
  }
}

void MyTemplateMatrix_return_ptrs_73(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyTemplateMatrix_templatedMethod_74(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodMatrix",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Matrix t = unwrap< Matrix >(in[1]);
  out[0] = wrap< Matrix >(obj->templatedMethod<gtsam::Matrix>(t));
}

void MyTemplateMatrix_templatedMethod_75(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint2",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Point2 t = unwrap< Point2 >(in[1]);
  out[0] = wrap< Point2 >(obj->templatedMethod<gtsam::Point2>(t));
}

void MyTemplateMatrix_templatedMethod_76(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodPoint3",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Point3 t = unwrap< Point3 >(in[1]);
  out[0] = wrap< Point3 >(obj->templatedMethod<gtsam::Point3>(t));
}

void MyTemplateMatrix_templatedMethod_77(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("templatedMethodVector",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<MyTemplate<gtsam::Matrix>>(in[0], "ptr_MyTemplateMatrix");
  Vector t = unwrap< Vector >(in[1]);
  out[0] = wrap< Vector >(obj->templatedMethod<gtsam::Vector>(t));
}

void MyTemplateMatrix_Level_78(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MyTemplateMatrix.Level",nargout,nargin,1);
  Matrix K = unwrap< Matrix >(in[0]);
  out[0] = wrap_shared_ptr(boost::make_shared<MyTemplate<Matrix>>(MyTemplate<gtsam::Matrix>::Level(K)),"MyTemplateMatrix", false);
}

void PrimitiveRefdouble_collectorInsertAndMakeBase_79(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<PrimitiveRef<double>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_PrimitiveRefdouble.insert(self);
}

void PrimitiveRefdouble_constructor_80(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<PrimitiveRef<double>> Shared;

  Shared *self = new Shared(new PrimitiveRef<double>());
  collector_PrimitiveRefdouble.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void PrimitiveRefdouble_deconstructor_81(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<PrimitiveRef<double>> Shared;
  checkArguments("delete_PrimitiveRefdouble",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_PrimitiveRefdouble::iterator item;
  item = collector_PrimitiveRefdouble.find(self);
  if(item != collector_PrimitiveRefdouble.end()) {
    delete self;
    collector_PrimitiveRefdouble.erase(item);
  }
}

void PrimitiveRefdouble_Brutal_82(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("PrimitiveRefdouble.Brutal",nargout,nargin,1);
  double t = unwrap< double >(in[0]);
  out[0] = wrap_shared_ptr(boost::make_shared<PrimitiveRef<double>>(PrimitiveRef<double>::Brutal(t)),"PrimitiveRefdouble", false);
}

void MyVector3_collectorInsertAndMakeBase_83(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<3>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyVector3.insert(self);
}

void MyVector3_constructor_84(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<3>> Shared;

  Shared *self = new Shared(new MyVector<3>());
  collector_MyVector3.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void MyVector3_deconstructor_85(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyVector12_collectorInsertAndMakeBase_86(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<12>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyVector12.insert(self);
}

void MyVector12_constructor_87(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyVector<12>> Shared;

  Shared *self = new Shared(new MyVector<12>());
  collector_MyVector12.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void MyVector12_deconstructor_88(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyFactorPosePoint2_collectorInsertAndMakeBase_89(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_MyFactorPosePoint2.insert(self);
}

void MyFactorPosePoint2_constructor_90(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void MyFactorPosePoint2_deconstructor_91(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void load2D_92(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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
void load2D_93(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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
void load2D_94(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("load2D",nargout,nargin,2);
  string filename = unwrap< string >(in[0]);
  boost::shared_ptr<gtsam::noiseModel::Diagonal> model = unwrap_shared_ptr< gtsam::noiseModel::Diagonal >(in[1], "ptr_gtsamnoiseModelDiagonal");
  auto pairResult = load2D(filename,model);
  out[0] = wrap_shared_ptr(pairResult.first,"gtsam.NonlinearFactorGraph", false);
  out[1] = wrap_shared_ptr(pairResult.second,"gtsam.Values", false);
}
void aGlobalFunction_95(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("aGlobalFunction",nargout,nargin,0);
  out[0] = wrap< Vector >(aGlobalFunction());
}
void overloadedGlobalFunction_96(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("overloadedGlobalFunction",nargout,nargin,1);
  int a = unwrap< int >(in[0]);
  out[0] = wrap< Vector >(overloadedGlobalFunction(a));
}
void overloadedGlobalFunction_97(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("overloadedGlobalFunction",nargout,nargin,2);
  int a = unwrap< int >(in[0]);
  double b = unwrap< double >(in[1]);
  out[0] = wrap< Vector >(overloadedGlobalFunction(a,b));
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
      gtsamPoint2_argUChar_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      gtsamPoint2_dim_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      gtsamPoint2_eigenArguments_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      gtsamPoint2_returnChar_8(nargout, out, nargin-1, in+1);
      break;
    case 9:
      gtsamPoint2_vectorConfusion_9(nargout, out, nargin-1, in+1);
      break;
    case 10:
      gtsamPoint2_x_10(nargout, out, nargin-1, in+1);
      break;
    case 11:
      gtsamPoint2_y_11(nargout, out, nargin-1, in+1);
      break;
    case 12:
      gtsamPoint3_collectorInsertAndMakeBase_12(nargout, out, nargin-1, in+1);
      break;
    case 13:
      gtsamPoint3_constructor_13(nargout, out, nargin-1, in+1);
      break;
    case 14:
      gtsamPoint3_deconstructor_14(nargout, out, nargin-1, in+1);
      break;
    case 15:
      gtsamPoint3_norm_15(nargout, out, nargin-1, in+1);
      break;
    case 16:
      gtsamPoint3_string_serialize_16(nargout, out, nargin-1, in+1);
      break;
    case 17:
      gtsamPoint3_StaticFunctionRet_17(nargout, out, nargin-1, in+1);
      break;
    case 18:
      gtsamPoint3_staticFunction_18(nargout, out, nargin-1, in+1);
      break;
    case 19:
      gtsamPoint3_string_deserialize_19(nargout, out, nargin-1, in+1);
      break;
    case 20:
      Test_collectorInsertAndMakeBase_20(nargout, out, nargin-1, in+1);
      break;
    case 21:
      Test_constructor_21(nargout, out, nargin-1, in+1);
      break;
    case 22:
      Test_constructor_22(nargout, out, nargin-1, in+1);
      break;
    case 23:
      Test_deconstructor_23(nargout, out, nargin-1, in+1);
      break;
    case 24:
      Test_arg_EigenConstRef_24(nargout, out, nargin-1, in+1);
      break;
    case 25:
      Test_create_MixedPtrs_25(nargout, out, nargin-1, in+1);
      break;
    case 26:
      Test_create_ptrs_26(nargout, out, nargin-1, in+1);
      break;
    case 27:
      Test_print_27(nargout, out, nargin-1, in+1);
      break;
    case 28:
      Test_return_Point2Ptr_28(nargout, out, nargin-1, in+1);
      break;
    case 29:
      Test_return_Test_29(nargout, out, nargin-1, in+1);
      break;
    case 30:
      Test_return_TestPtr_30(nargout, out, nargin-1, in+1);
      break;
    case 31:
      Test_return_bool_31(nargout, out, nargin-1, in+1);
      break;
    case 32:
      Test_return_double_32(nargout, out, nargin-1, in+1);
      break;
    case 33:
      Test_return_field_33(nargout, out, nargin-1, in+1);
      break;
    case 34:
      Test_return_int_34(nargout, out, nargin-1, in+1);
      break;
    case 35:
      Test_return_matrix1_35(nargout, out, nargin-1, in+1);
      break;
    case 36:
      Test_return_matrix2_36(nargout, out, nargin-1, in+1);
      break;
    case 37:
      Test_return_pair_37(nargout, out, nargin-1, in+1);
      break;
    case 38:
      Test_return_pair_38(nargout, out, nargin-1, in+1);
      break;
    case 39:
      Test_return_ptrs_39(nargout, out, nargin-1, in+1);
      break;
    case 40:
      Test_return_size_t_40(nargout, out, nargin-1, in+1);
      break;
    case 41:
      Test_return_string_41(nargout, out, nargin-1, in+1);
      break;
    case 42:
      Test_return_vector1_42(nargout, out, nargin-1, in+1);
      break;
    case 43:
      Test_return_vector2_43(nargout, out, nargin-1, in+1);
      break;
    case 44:
      MyBase_collectorInsertAndMakeBase_44(nargout, out, nargin-1, in+1);
      break;
    case 45:
      MyBase_upcastFromVoid_45(nargout, out, nargin-1, in+1);
      break;
    case 46:
      MyBase_deconstructor_46(nargout, out, nargin-1, in+1);
      break;
    case 47:
      MyTemplatePoint2_collectorInsertAndMakeBase_47(nargout, out, nargin-1, in+1);
      break;
    case 48:
      MyTemplatePoint2_upcastFromVoid_48(nargout, out, nargin-1, in+1);
      break;
    case 49:
      MyTemplatePoint2_constructor_49(nargout, out, nargin-1, in+1);
      break;
    case 50:
      MyTemplatePoint2_deconstructor_50(nargout, out, nargin-1, in+1);
      break;
    case 51:
      MyTemplatePoint2_accept_T_51(nargout, out, nargin-1, in+1);
      break;
    case 52:
      MyTemplatePoint2_accept_Tptr_52(nargout, out, nargin-1, in+1);
      break;
    case 53:
      MyTemplatePoint2_create_MixedPtrs_53(nargout, out, nargin-1, in+1);
      break;
    case 54:
      MyTemplatePoint2_create_ptrs_54(nargout, out, nargin-1, in+1);
      break;
    case 55:
      MyTemplatePoint2_return_T_55(nargout, out, nargin-1, in+1);
      break;
    case 56:
      MyTemplatePoint2_return_Tptr_56(nargout, out, nargin-1, in+1);
      break;
    case 57:
      MyTemplatePoint2_return_ptrs_57(nargout, out, nargin-1, in+1);
      break;
    case 58:
      MyTemplatePoint2_templatedMethod_58(nargout, out, nargin-1, in+1);
      break;
    case 59:
      MyTemplatePoint2_templatedMethod_59(nargout, out, nargin-1, in+1);
      break;
    case 60:
      MyTemplatePoint2_templatedMethod_60(nargout, out, nargin-1, in+1);
      break;
    case 61:
      MyTemplatePoint2_templatedMethod_61(nargout, out, nargin-1, in+1);
      break;
    case 62:
      MyTemplatePoint2_Level_62(nargout, out, nargin-1, in+1);
      break;
    case 63:
      MyTemplateMatrix_collectorInsertAndMakeBase_63(nargout, out, nargin-1, in+1);
      break;
    case 64:
      MyTemplateMatrix_upcastFromVoid_64(nargout, out, nargin-1, in+1);
      break;
    case 65:
      MyTemplateMatrix_constructor_65(nargout, out, nargin-1, in+1);
      break;
    case 66:
      MyTemplateMatrix_deconstructor_66(nargout, out, nargin-1, in+1);
      break;
    case 67:
      MyTemplateMatrix_accept_T_67(nargout, out, nargin-1, in+1);
      break;
    case 68:
      MyTemplateMatrix_accept_Tptr_68(nargout, out, nargin-1, in+1);
      break;
    case 69:
      MyTemplateMatrix_create_MixedPtrs_69(nargout, out, nargin-1, in+1);
      break;
    case 70:
      MyTemplateMatrix_create_ptrs_70(nargout, out, nargin-1, in+1);
      break;
    case 71:
      MyTemplateMatrix_return_T_71(nargout, out, nargin-1, in+1);
      break;
    case 72:
      MyTemplateMatrix_return_Tptr_72(nargout, out, nargin-1, in+1);
      break;
    case 73:
      MyTemplateMatrix_return_ptrs_73(nargout, out, nargin-1, in+1);
      break;
    case 74:
      MyTemplateMatrix_templatedMethod_74(nargout, out, nargin-1, in+1);
      break;
    case 75:
      MyTemplateMatrix_templatedMethod_75(nargout, out, nargin-1, in+1);
      break;
    case 76:
      MyTemplateMatrix_templatedMethod_76(nargout, out, nargin-1, in+1);
      break;
    case 77:
      MyTemplateMatrix_templatedMethod_77(nargout, out, nargin-1, in+1);
      break;
    case 78:
      MyTemplateMatrix_Level_78(nargout, out, nargin-1, in+1);
      break;
    case 79:
      PrimitiveRefdouble_collectorInsertAndMakeBase_79(nargout, out, nargin-1, in+1);
      break;
    case 80:
      PrimitiveRefdouble_constructor_80(nargout, out, nargin-1, in+1);
      break;
    case 81:
      PrimitiveRefdouble_deconstructor_81(nargout, out, nargin-1, in+1);
      break;
    case 82:
      PrimitiveRefdouble_Brutal_82(nargout, out, nargin-1, in+1);
      break;
    case 83:
      MyVector3_collectorInsertAndMakeBase_83(nargout, out, nargin-1, in+1);
      break;
    case 84:
      MyVector3_constructor_84(nargout, out, nargin-1, in+1);
      break;
    case 85:
      MyVector3_deconstructor_85(nargout, out, nargin-1, in+1);
      break;
    case 86:
      MyVector12_collectorInsertAndMakeBase_86(nargout, out, nargin-1, in+1);
      break;
    case 87:
      MyVector12_constructor_87(nargout, out, nargin-1, in+1);
      break;
    case 88:
      MyVector12_deconstructor_88(nargout, out, nargin-1, in+1);
      break;
    case 89:
      MyFactorPosePoint2_collectorInsertAndMakeBase_89(nargout, out, nargin-1, in+1);
      break;
    case 90:
      MyFactorPosePoint2_constructor_90(nargout, out, nargin-1, in+1);
      break;
    case 91:
      MyFactorPosePoint2_deconstructor_91(nargout, out, nargin-1, in+1);
      break;
    case 92:
      load2D_92(nargout, out, nargin-1, in+1);
      break;
    case 93:
      load2D_93(nargout, out, nargin-1, in+1);
      break;
    case 94:
      load2D_94(nargout, out, nargin-1, in+1);
      break;
    case 95:
      aGlobalFunction_95(nargout, out, nargin-1, in+1);
      break;
    case 96:
      overloadedGlobalFunction_96(nargout, out, nargin-1, in+1);
      break;
    case 97:
      overloadedGlobalFunction_97(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

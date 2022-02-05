#include <gtwrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>


BOOST_CLASS_EXPORT_GUID(gtsam::Point2, "gtsamPoint2");
BOOST_CLASS_EXPORT_GUID(gtsam::Point3, "gtsamPoint3");

typedef std::set<boost::shared_ptr<gtsam::Point2>*> Collector_gtsamPoint2;
static Collector_gtsamPoint2 collector_gtsamPoint2;
typedef std::set<boost::shared_ptr<gtsam::Point3>*> Collector_gtsamPoint3;
static Collector_gtsamPoint3 collector_gtsamPoint3;


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
    collector_gtsamPoint2.erase(item);
  }
  delete self;
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
  char* a = unwrap_ptr< char >(in[1], "ptr_char");
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
  char* a = unwrap_ptr< char >(in[1], "ptr_char");
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
    collector_gtsamPoint3.erase(item);
  }
  delete self;
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
  checkArguments("gtsam::Point3.StaticFunctionRet",nargout,nargin,1);
  double z = unwrap< double >(in[0]);
  out[0] = wrap< Point3 >(gtsam::Point3::StaticFunctionRet(z));
}

void gtsamPoint3_staticFunction_24(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("gtsam::Point3.staticFunction",nargout,nargin,0);
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
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

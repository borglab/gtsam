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

void _functions_RTTIRegister() {
  const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_functions_rttiRegistry_created");
  if(!alreadyCreated) {
    std::map<std::string, std::string> types;

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

void load2D_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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
void load2D_1(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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
void load2D_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("load2D",nargout,nargin,2);
  string filename = unwrap< string >(in[0]);
  boost::shared_ptr<gtsam::noiseModel::Diagonal> model = unwrap_shared_ptr< gtsam::noiseModel::Diagonal >(in[1], "ptr_gtsamnoiseModelDiagonal");
  auto pairResult = load2D(filename,model);
  out[0] = wrap_shared_ptr(pairResult.first,"gtsam.NonlinearFactorGraph", false);
  out[1] = wrap_shared_ptr(pairResult.second,"gtsam.Values", false);
}
void aGlobalFunction_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("aGlobalFunction",nargout,nargin,0);
  out[0] = wrap< Vector >(aGlobalFunction());
}
void overloadedGlobalFunction_4(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("overloadedGlobalFunction",nargout,nargin,1);
  int a = unwrap< int >(in[0]);
  out[0] = wrap< Vector >(overloadedGlobalFunction(a));
}
void overloadedGlobalFunction_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("overloadedGlobalFunction",nargout,nargin,2);
  int a = unwrap< int >(in[0]);
  double b = unwrap< double >(in[1]);
  out[0] = wrap< Vector >(overloadedGlobalFunction(a,b));
}
void MultiTemplatedFunctionStringSize_tDouble_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MultiTemplatedFunctionStringSize_tDouble",nargout,nargin,2);
  T& x = *unwrap_shared_ptr< T >(in[0], "ptr_T");
  size_t y = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(MultiTemplatedFunctionStringSize_tDouble(x,y));
}
void MultiTemplatedFunctionDoubleSize_tDouble_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("MultiTemplatedFunctionDoubleSize_tDouble",nargout,nargin,2);
  T& x = *unwrap_shared_ptr< T >(in[0], "ptr_T");
  size_t y = unwrap< size_t >(in[1]);
  out[0] = wrap< double >(MultiTemplatedFunctionDoubleSize_tDouble(x,y));
}
void TemplatedFunctionRot3_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("TemplatedFunctionRot3",nargout,nargin,1);
  gtsam::Rot3& t = *unwrap_shared_ptr< gtsam::Rot3 >(in[0], "ptr_gtsamRot3");
  TemplatedFunctionRot3(t);
}

void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  _functions_RTTIRegister();

  int id = unwrap<int>(in[0]);

  try {
    switch(id) {
    case 0:
      load2D_0(nargout, out, nargin-1, in+1);
      break;
    case 1:
      load2D_1(nargout, out, nargin-1, in+1);
      break;
    case 2:
      load2D_2(nargout, out, nargin-1, in+1);
      break;
    case 3:
      aGlobalFunction_3(nargout, out, nargin-1, in+1);
      break;
    case 4:
      overloadedGlobalFunction_4(nargout, out, nargin-1, in+1);
      break;
    case 5:
      overloadedGlobalFunction_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      MultiTemplatedFunctionStringSize_tDouble_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      MultiTemplatedFunctionDoubleSize_tDouble_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      TemplatedFunctionRot3_8(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

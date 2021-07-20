#include <gtwrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>







void _deleteAllObjects()
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  bool anyDeleted = false;

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
void DefaultFuncInt_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncInt",nargout,nargin,2);
  int a = unwrap< int >(in[0]);
  int b = unwrap< int >(in[1]);
  DefaultFuncInt(a,b);
}
void DefaultFuncString_9(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncString",nargout,nargin,2);
  string& s = *unwrap_shared_ptr< string >(in[0], "ptr_string");
  string& name = *unwrap_shared_ptr< string >(in[1], "ptr_string");
  DefaultFuncString(s,name);
}
void DefaultFuncObj_10(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncObj",nargout,nargin,1);
  gtsam::KeyFormatter& keyFormatter = *unwrap_shared_ptr< gtsam::KeyFormatter >(in[0], "ptr_gtsamKeyFormatter");
  DefaultFuncObj(keyFormatter);
}
void DefaultFuncZero_11(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncZero",nargout,nargin,5);
  int a = unwrap< int >(in[0]);
  int b = unwrap< int >(in[1]);
  double c = unwrap< double >(in[2]);
  bool d = unwrap< bool >(in[3]);
  bool e = unwrap< bool >(in[4]);
  DefaultFuncZero(a,b,c,d,e);
}
void DefaultFuncVector_12(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncVector",nargout,nargin,2);
  std::vector<int>& i = *unwrap_shared_ptr< std::vector<int> >(in[0], "ptr_stdvectorint");
  std::vector<string>& s = *unwrap_shared_ptr< std::vector<string> >(in[1], "ptr_stdvectorstring");
  DefaultFuncVector(i,s);
}
void setPose_13(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("setPose",nargout,nargin,1);
  gtsam::Pose3& pose = *unwrap_shared_ptr< gtsam::Pose3 >(in[0], "ptr_gtsamPose3");
  setPose(pose);
}
void TemplatedFunctionRot3_14(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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
      DefaultFuncInt_8(nargout, out, nargin-1, in+1);
      break;
    case 9:
      DefaultFuncString_9(nargout, out, nargin-1, in+1);
      break;
    case 10:
      DefaultFuncObj_10(nargout, out, nargin-1, in+1);
      break;
    case 11:
      DefaultFuncZero_11(nargout, out, nargin-1, in+1);
      break;
    case 12:
      DefaultFuncVector_12(nargout, out, nargin-1, in+1);
      break;
    case 13:
      setPose_13(nargout, out, nargin-1, in+1);
      break;
    case 14:
      TemplatedFunctionRot3_14(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

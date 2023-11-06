#include <gtwrap/matlab.h>
#include <map>







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
    if(mexPutVariable("global", "gtsam_functions_rttiRegistry_created", newAlreadyCreated) != 0) {
      mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
    }
    mxDestroyArray(newAlreadyCreated);
  }
}

void load2D_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("load2D",nargout,nargin,5);
  string filename = unwrap< string >(in[0]);
  std::shared_ptr<Test> model = unwrap_shared_ptr< Test >(in[1], "ptr_Test");
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
  std::shared_ptr<gtsam::noiseModel::Diagonal> model = unwrap_shared_ptr< gtsam::noiseModel::Diagonal >(in[1], "ptr_gtsamnoiseModelDiagonal");
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
  gtsam::noiseModel::Diagonal* model = unwrap_ptr< gtsam::noiseModel::Diagonal >(in[1], "ptr_gtsamnoiseModelDiagonal");
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
void DefaultFuncInt_9(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncInt",nargout,nargin,1);
  int a = unwrap< int >(in[0]);
  DefaultFuncInt(a,0);
}
void DefaultFuncInt_10(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncInt",nargout,nargin,0);
  DefaultFuncInt(123,0);
}
void DefaultFuncString_11(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncString",nargout,nargin,2);
  string& s = *unwrap_shared_ptr< string >(in[0], "ptr_string");
  string& name = *unwrap_shared_ptr< string >(in[1], "ptr_string");
  DefaultFuncString(s,name);
}
void DefaultFuncString_12(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncString",nargout,nargin,1);
  string& s = *unwrap_shared_ptr< string >(in[0], "ptr_string");
  DefaultFuncString(s,"");
}
void DefaultFuncString_13(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncString",nargout,nargin,0);
  DefaultFuncString("hello","");
}
void DefaultFuncObj_14(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncObj",nargout,nargin,1);
  gtsam::KeyFormatter& keyFormatter = *unwrap_shared_ptr< gtsam::KeyFormatter >(in[0], "ptr_gtsamKeyFormatter");
  DefaultFuncObj(keyFormatter);
}
void DefaultFuncObj_15(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncObj",nargout,nargin,0);
  DefaultFuncObj(gtsam::DefaultKeyFormatter);
}
void DefaultFuncZero_16(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncZero",nargout,nargin,5);
  int a = unwrap< int >(in[0]);
  int b = unwrap< int >(in[1]);
  double c = unwrap< double >(in[2]);
  int d = unwrap< int >(in[3]);
  bool e = unwrap< bool >(in[4]);
  DefaultFuncZero(a,b,c,d,e);
}
void DefaultFuncZero_17(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncZero",nargout,nargin,4);
  int a = unwrap< int >(in[0]);
  int b = unwrap< int >(in[1]);
  double c = unwrap< double >(in[2]);
  int d = unwrap< int >(in[3]);
  DefaultFuncZero(a,b,c,d,false);
}
void DefaultFuncZero_18(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncZero",nargout,nargin,3);
  int a = unwrap< int >(in[0]);
  int b = unwrap< int >(in[1]);
  double c = unwrap< double >(in[2]);
  DefaultFuncZero(a,b,c,0,false);
}
void DefaultFuncZero_19(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncZero",nargout,nargin,2);
  int a = unwrap< int >(in[0]);
  int b = unwrap< int >(in[1]);
  DefaultFuncZero(a,b,0.0,0,false);
}
void DefaultFuncVector_20(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncVector",nargout,nargin,2);
  std::vector<int>& i = *unwrap_shared_ptr< std::vector<int> >(in[0], "ptr_stdvectorint");
  std::vector<string>& s = *unwrap_shared_ptr< std::vector<string> >(in[1], "ptr_stdvectorstring");
  DefaultFuncVector(i,s);
}
void DefaultFuncVector_21(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncVector",nargout,nargin,1);
  std::vector<int>& i = *unwrap_shared_ptr< std::vector<int> >(in[0], "ptr_stdvectorint");
  DefaultFuncVector(i,{"borglab", "gtsam"});
}
void DefaultFuncVector_22(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("DefaultFuncVector",nargout,nargin,0);
  DefaultFuncVector({1, 2, 3},{"borglab", "gtsam"});
}
void setPose_23(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("setPose",nargout,nargin,1);
  gtsam::Pose3& pose = *unwrap_shared_ptr< gtsam::Pose3 >(in[0], "ptr_gtsamPose3");
  setPose(pose);
}
void setPose_24(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("setPose",nargout,nargin,0);
  setPose(gtsam::Pose3());
}
void EliminateDiscrete_25(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("EliminateDiscrete",nargout,nargin,2);
  gtsam::DiscreteFactorGraph& factors = *unwrap_shared_ptr< gtsam::DiscreteFactorGraph >(in[0], "ptr_gtsamDiscreteFactorGraph");
  gtsam::Ordering& frontalKeys = *unwrap_shared_ptr< gtsam::Ordering >(in[1], "ptr_gtsamOrdering");
  auto pairResult = EliminateDiscrete(factors,frontalKeys);
  out[0] = wrap_shared_ptr(pairResult.first,"gtsam.DiscreteConditional", false);
  out[1] = wrap_shared_ptr(pairResult.second,"gtsam.DecisionTreeFactor", false);
}
void TemplatedFunctionRot3_26(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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
      DefaultFuncInt_9(nargout, out, nargin-1, in+1);
      break;
    case 10:
      DefaultFuncInt_10(nargout, out, nargin-1, in+1);
      break;
    case 11:
      DefaultFuncString_11(nargout, out, nargin-1, in+1);
      break;
    case 12:
      DefaultFuncString_12(nargout, out, nargin-1, in+1);
      break;
    case 13:
      DefaultFuncString_13(nargout, out, nargin-1, in+1);
      break;
    case 14:
      DefaultFuncObj_14(nargout, out, nargin-1, in+1);
      break;
    case 15:
      DefaultFuncObj_15(nargout, out, nargin-1, in+1);
      break;
    case 16:
      DefaultFuncZero_16(nargout, out, nargin-1, in+1);
      break;
    case 17:
      DefaultFuncZero_17(nargout, out, nargin-1, in+1);
      break;
    case 18:
      DefaultFuncZero_18(nargout, out, nargin-1, in+1);
      break;
    case 19:
      DefaultFuncZero_19(nargout, out, nargin-1, in+1);
      break;
    case 20:
      DefaultFuncVector_20(nargout, out, nargin-1, in+1);
      break;
    case 21:
      DefaultFuncVector_21(nargout, out, nargin-1, in+1);
      break;
    case 22:
      DefaultFuncVector_22(nargout, out, nargin-1, in+1);
      break;
    case 23:
      setPose_23(nargout, out, nargin-1, in+1);
      break;
    case 24:
      setPose_24(nargout, out, nargin-1, in+1);
      break;
    case 25:
      EliminateDiscrete_25(nargout, out, nargin-1, in+1);
      break;
    case 26:
      TemplatedFunctionRot3_26(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

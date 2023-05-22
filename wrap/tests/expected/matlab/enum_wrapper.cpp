#include <gtwrap/matlab.h>
#include <map>



typedef gtsam::Optimizer<gtsam::GaussNewtonParams> OptimizerGaussNewtonParams;

typedef std::set<std::shared_ptr<Pet>*> Collector_Pet;
static Collector_Pet collector_Pet;
typedef std::set<std::shared_ptr<gtsam::MCU>*> Collector_gtsamMCU;
static Collector_gtsamMCU collector_gtsamMCU;
typedef std::set<std::shared_ptr<OptimizerGaussNewtonParams>*> Collector_gtsamOptimizerGaussNewtonParams;
static Collector_gtsamOptimizerGaussNewtonParams collector_gtsamOptimizerGaussNewtonParams;


void _deleteAllObjects()
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  bool anyDeleted = false;
  { for(Collector_Pet::iterator iter = collector_Pet.begin();
      iter != collector_Pet.end(); ) {
    delete *iter;
    collector_Pet.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gtsamMCU::iterator iter = collector_gtsamMCU.begin();
      iter != collector_gtsamMCU.end(); ) {
    delete *iter;
    collector_gtsamMCU.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gtsamOptimizerGaussNewtonParams::iterator iter = collector_gtsamOptimizerGaussNewtonParams.begin();
      iter != collector_gtsamOptimizerGaussNewtonParams.end(); ) {
    delete *iter;
    collector_gtsamOptimizerGaussNewtonParams.erase(iter++);
    anyDeleted = true;
  } }

  if(anyDeleted)
    cout <<
      "WARNING:  Wrap modules with variables in the workspace have been reloaded due to\n"
      "calling destructors, call 'clear all' again if you plan to now recompile a wrap\n"
      "module, so that your recompiled module is used instead of the old one." << endl;
  std::cout.rdbuf(outbuf);
}

void _enum_RTTIRegister() {
  const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_enum_rttiRegistry_created");
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
    if(mexPutVariable("global", "gtsam_enum_rttiRegistry_created", newAlreadyCreated) != 0) {
      mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
    }
    mxDestroyArray(newAlreadyCreated);
  }
}

void Pet_collectorInsertAndMakeBase_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef std::shared_ptr<Pet> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_Pet.insert(self);
}

void Pet_constructor_1(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef std::shared_ptr<Pet> Shared;

  string& name = *unwrap_shared_ptr< string >(in[0], "ptr_string");
  std::shared_ptr<Pet::Kind> type = unwrap_enum<Pet::Kind>(in[1]);
  Shared *self = new Shared(new Pet(name,*type));
  collector_Pet.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void Pet_deconstructor_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef std::shared_ptr<Pet> Shared;
  checkArguments("delete_Pet",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_Pet::iterator item;
  item = collector_Pet.find(self);
  if(item != collector_Pet.end()) {
    collector_Pet.erase(item);
  }
  delete self;
}

void Pet_get_name_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("name",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Pet>(in[0], "ptr_Pet");
  out[0] = wrap< string >(obj->name);
}

void Pet_set_name_4(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("name",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Pet>(in[0], "ptr_Pet");
  string name = unwrap< string >(in[1]);
  obj->name = name;
}

void Pet_get_type_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("type",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<Pet>(in[0], "ptr_Pet");
  out[0] = wrap_enum(obj->type,"Pet.Kind");
}

void Pet_set_type_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("type",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<Pet>(in[0], "ptr_Pet");
  std::shared_ptr<Pet::Kind> type = unwrap_enum<Pet::Kind>(in[1]);
  obj->type = *type;
}

void gtsamMCU_collectorInsertAndMakeBase_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef std::shared_ptr<gtsam::MCU> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamMCU.insert(self);
}

void gtsamMCU_constructor_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef std::shared_ptr<gtsam::MCU> Shared;

  Shared *self = new Shared(new gtsam::MCU());
  collector_gtsamMCU.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gtsamMCU_deconstructor_9(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef std::shared_ptr<gtsam::MCU> Shared;
  checkArguments("delete_gtsamMCU",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamMCU::iterator item;
  item = collector_gtsamMCU.find(self);
  if(item != collector_gtsamMCU.end()) {
    collector_gtsamMCU.erase(item);
  }
  delete self;
}

void gtsamOptimizerGaussNewtonParams_collectorInsertAndMakeBase_10(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef std::shared_ptr<gtsam::Optimizer<gtsam::GaussNewtonParams>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamOptimizerGaussNewtonParams.insert(self);
}

void gtsamOptimizerGaussNewtonParams_deconstructor_11(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef std::shared_ptr<gtsam::Optimizer<gtsam::GaussNewtonParams>> Shared;
  checkArguments("delete_gtsamOptimizerGaussNewtonParams",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamOptimizerGaussNewtonParams::iterator item;
  item = collector_gtsamOptimizerGaussNewtonParams.find(self);
  if(item != collector_gtsamOptimizerGaussNewtonParams.end()) {
    collector_gtsamOptimizerGaussNewtonParams.erase(item);
  }
  delete self;
}

void gtsamOptimizerGaussNewtonParams_setVerbosity_12(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("setVerbosity",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::Optimizer<gtsam::GaussNewtonParams>>(in[0], "ptr_gtsamOptimizerGaussNewtonParams");
  std::shared_ptr<Optimizer<gtsam::GaussNewtonParams>::Verbosity> value = unwrap_enum<Optimizer<gtsam::GaussNewtonParams>::Verbosity>(in[1]);
  obj->setVerbosity(*value);
}


void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  _enum_RTTIRegister();

  int id = unwrap<int>(in[0]);

  try {
    switch(id) {
    case 0:
      Pet_collectorInsertAndMakeBase_0(nargout, out, nargin-1, in+1);
      break;
    case 1:
      Pet_constructor_1(nargout, out, nargin-1, in+1);
      break;
    case 2:
      Pet_deconstructor_2(nargout, out, nargin-1, in+1);
      break;
    case 3:
      Pet_get_name_3(nargout, out, nargin-1, in+1);
      break;
    case 4:
      Pet_set_name_4(nargout, out, nargin-1, in+1);
      break;
    case 5:
      Pet_get_type_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      Pet_set_type_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      gtsamMCU_collectorInsertAndMakeBase_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      gtsamMCU_constructor_8(nargout, out, nargin-1, in+1);
      break;
    case 9:
      gtsamMCU_deconstructor_9(nargout, out, nargin-1, in+1);
      break;
    case 10:
      gtsamOptimizerGaussNewtonParams_collectorInsertAndMakeBase_10(nargout, out, nargin-1, in+1);
      break;
    case 11:
      gtsamOptimizerGaussNewtonParams_deconstructor_11(nargout, out, nargin-1, in+1);
      break;
    case 12:
      gtsamOptimizerGaussNewtonParams_setVerbosity_12(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

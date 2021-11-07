#include <gtwrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>



typedef ScopedTemplate<Result> ScopedTemplateResult;

typedef std::set<boost::shared_ptr<TemplatedConstructor>*> Collector_TemplatedConstructor;
static Collector_TemplatedConstructor collector_TemplatedConstructor;
typedef std::set<boost::shared_ptr<ScopedTemplateResult>*> Collector_ScopedTemplateResult;
static Collector_ScopedTemplateResult collector_ScopedTemplateResult;


void _deleteAllObjects()
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  bool anyDeleted = false;
  { for(Collector_TemplatedConstructor::iterator iter = collector_TemplatedConstructor.begin();
      iter != collector_TemplatedConstructor.end(); ) {
    delete *iter;
    collector_TemplatedConstructor.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_ScopedTemplateResult::iterator iter = collector_ScopedTemplateResult.begin();
      iter != collector_ScopedTemplateResult.end(); ) {
    delete *iter;
    collector_ScopedTemplateResult.erase(iter++);
    anyDeleted = true;
  } }

  if(anyDeleted)
    cout <<
      "WARNING:  Wrap modules with variables in the workspace have been reloaded due to\n"
      "calling destructors, call 'clear all' again if you plan to now recompile a wrap\n"
      "module, so that your recompiled module is used instead of the old one." << endl;
  std::cout.rdbuf(outbuf);
}

void _template_RTTIRegister() {
  const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_template_rttiRegistry_created");
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

void TemplatedConstructor_collectorInsertAndMakeBase_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_TemplatedConstructor.insert(self);
}

void TemplatedConstructor_constructor_1(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  Shared *self = new Shared(new TemplatedConstructor());
  collector_TemplatedConstructor.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void TemplatedConstructor_constructor_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  string& arg = *unwrap_shared_ptr< string >(in[0], "ptr_string");
  Shared *self = new Shared(new TemplatedConstructor(arg));
  collector_TemplatedConstructor.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void TemplatedConstructor_constructor_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  int arg = unwrap< int >(in[0]);
  Shared *self = new Shared(new TemplatedConstructor(arg));
  collector_TemplatedConstructor.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void TemplatedConstructor_constructor_4(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<TemplatedConstructor> Shared;

  double arg = unwrap< double >(in[0]);
  Shared *self = new Shared(new TemplatedConstructor(arg));
  collector_TemplatedConstructor.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void TemplatedConstructor_deconstructor_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
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

void ScopedTemplateResult_collectorInsertAndMakeBase_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<ScopedTemplate<Result>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_ScopedTemplateResult.insert(self);
}

void ScopedTemplateResult_constructor_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<ScopedTemplate<Result>> Shared;

  Result::Value& arg = *unwrap_shared_ptr< Result::Value >(in[0], "ptr_Result::Value");
  Shared *self = new Shared(new ScopedTemplate<Result>(arg));
  collector_ScopedTemplateResult.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void ScopedTemplateResult_deconstructor_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<ScopedTemplate<Result>> Shared;
  checkArguments("delete_ScopedTemplateResult",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_ScopedTemplateResult::iterator item;
  item = collector_ScopedTemplateResult.find(self);
  if(item != collector_ScopedTemplateResult.end()) {
    delete self;
    collector_ScopedTemplateResult.erase(item);
  }
}


void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  _template_RTTIRegister();

  int id = unwrap<int>(in[0]);

  try {
    switch(id) {
    case 0:
      TemplatedConstructor_collectorInsertAndMakeBase_0(nargout, out, nargin-1, in+1);
      break;
    case 1:
      TemplatedConstructor_constructor_1(nargout, out, nargin-1, in+1);
      break;
    case 2:
      TemplatedConstructor_constructor_2(nargout, out, nargin-1, in+1);
      break;
    case 3:
      TemplatedConstructor_constructor_3(nargout, out, nargin-1, in+1);
      break;
    case 4:
      TemplatedConstructor_constructor_4(nargout, out, nargin-1, in+1);
      break;
    case 5:
      TemplatedConstructor_deconstructor_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      ScopedTemplateResult_collectorInsertAndMakeBase_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      ScopedTemplateResult_constructor_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      ScopedTemplateResult_deconstructor_8(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

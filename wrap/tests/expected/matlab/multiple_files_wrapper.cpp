#include <gtwrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>





typedef std::set<boost::shared_ptr<gtsam::Class1>*> Collector_gtsamClass1;
static Collector_gtsamClass1 collector_gtsamClass1;
typedef std::set<boost::shared_ptr<gtsam::Class2>*> Collector_gtsamClass2;
static Collector_gtsamClass2 collector_gtsamClass2;
typedef std::set<boost::shared_ptr<gtsam::ClassA>*> Collector_gtsamClassA;
static Collector_gtsamClassA collector_gtsamClassA;


void _deleteAllObjects()
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  bool anyDeleted = false;
  { for(Collector_gtsamClass1::iterator iter = collector_gtsamClass1.begin();
      iter != collector_gtsamClass1.end(); ) {
    delete *iter;
    collector_gtsamClass1.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gtsamClass2::iterator iter = collector_gtsamClass2.begin();
      iter != collector_gtsamClass2.end(); ) {
    delete *iter;
    collector_gtsamClass2.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gtsamClassA::iterator iter = collector_gtsamClassA.begin();
      iter != collector_gtsamClassA.end(); ) {
    delete *iter;
    collector_gtsamClassA.erase(iter++);
    anyDeleted = true;
  } }

  if(anyDeleted)
    cout <<
      "WARNING:  Wrap modules with variables in the workspace have been reloaded due to\n"
      "calling destructors, call 'clear all' again if you plan to now recompile a wrap\n"
      "module, so that your recompiled module is used instead of the old one." << endl;
  std::cout.rdbuf(outbuf);
}

void _multiple_files_RTTIRegister() {
  const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_multiple_files_rttiRegistry_created");
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

void gtsamClass1_collectorInsertAndMakeBase_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Class1> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamClass1.insert(self);
}

void gtsamClass1_constructor_1(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Class1> Shared;

  Shared *self = new Shared(new gtsam::Class1());
  collector_gtsamClass1.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gtsamClass1_deconstructor_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Class1> Shared;
  checkArguments("delete_gtsamClass1",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamClass1::iterator item;
  item = collector_gtsamClass1.find(self);
  if(item != collector_gtsamClass1.end()) {
    delete self;
    collector_gtsamClass1.erase(item);
  }
}

void gtsamClass2_collectorInsertAndMakeBase_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Class2> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamClass2.insert(self);
}

void gtsamClass2_constructor_4(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::Class2> Shared;

  Shared *self = new Shared(new gtsam::Class2());
  collector_gtsamClass2.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gtsamClass2_deconstructor_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::Class2> Shared;
  checkArguments("delete_gtsamClass2",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamClass2::iterator item;
  item = collector_gtsamClass2.find(self);
  if(item != collector_gtsamClass2.end()) {
    delete self;
    collector_gtsamClass2.erase(item);
  }
}

void gtsamClassA_collectorInsertAndMakeBase_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::ClassA> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamClassA.insert(self);
}

void gtsamClassA_constructor_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::ClassA> Shared;

  Shared *self = new Shared(new gtsam::ClassA());
  collector_gtsamClassA.insert(self);
  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
  *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
}

void gtsamClassA_deconstructor_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::ClassA> Shared;
  checkArguments("delete_gtsamClassA",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamClassA::iterator item;
  item = collector_gtsamClassA.find(self);
  if(item != collector_gtsamClassA.end()) {
    delete self;
    collector_gtsamClassA.erase(item);
  }
}


void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  _multiple_files_RTTIRegister();

  int id = unwrap<int>(in[0]);

  try {
    switch(id) {
    case 0:
      gtsamClass1_collectorInsertAndMakeBase_0(nargout, out, nargin-1, in+1);
      break;
    case 1:
      gtsamClass1_constructor_1(nargout, out, nargin-1, in+1);
      break;
    case 2:
      gtsamClass1_deconstructor_2(nargout, out, nargin-1, in+1);
      break;
    case 3:
      gtsamClass2_collectorInsertAndMakeBase_3(nargout, out, nargin-1, in+1);
      break;
    case 4:
      gtsamClass2_constructor_4(nargout, out, nargin-1, in+1);
      break;
    case 5:
      gtsamClass2_deconstructor_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      gtsamClassA_collectorInsertAndMakeBase_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      gtsamClassA_constructor_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      gtsamClassA_deconstructor_8(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

#include <gtwrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>

#include <gtsam/geometry/Cal3Bundler.h>

typedef gtsam::PinholeCamera<gtsam::Cal3Bundler> PinholeCameraCal3Bundler;
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3> GeneralSFMFactorCal3Bundler;

typedef std::set<std::shared_ptr<gtsam::NonlinearFactorGraph>*> Collector_gtsamNonlinearFactorGraph;
static Collector_gtsamNonlinearFactorGraph collector_gtsamNonlinearFactorGraph;
typedef std::set<std::shared_ptr<gtsam::SfmTrack>*> Collector_gtsamSfmTrack;
static Collector_gtsamSfmTrack collector_gtsamSfmTrack;
typedef std::set<std::shared_ptr<PinholeCameraCal3Bundler>*> Collector_gtsamPinholeCameraCal3Bundler;
static Collector_gtsamPinholeCameraCal3Bundler collector_gtsamPinholeCameraCal3Bundler;
typedef std::set<std::shared_ptr<GeneralSFMFactorCal3Bundler>*> Collector_gtsamGeneralSFMFactorCal3Bundler;
static Collector_gtsamGeneralSFMFactorCal3Bundler collector_gtsamGeneralSFMFactorCal3Bundler;


void _deleteAllObjects()
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  bool anyDeleted = false;
  { for(Collector_gtsamNonlinearFactorGraph::iterator iter = collector_gtsamNonlinearFactorGraph.begin();
      iter != collector_gtsamNonlinearFactorGraph.end(); ) {
    delete *iter;
    collector_gtsamNonlinearFactorGraph.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gtsamSfmTrack::iterator iter = collector_gtsamSfmTrack.begin();
      iter != collector_gtsamSfmTrack.end(); ) {
    delete *iter;
    collector_gtsamSfmTrack.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gtsamPinholeCameraCal3Bundler::iterator iter = collector_gtsamPinholeCameraCal3Bundler.begin();
      iter != collector_gtsamPinholeCameraCal3Bundler.end(); ) {
    delete *iter;
    collector_gtsamPinholeCameraCal3Bundler.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_gtsamGeneralSFMFactorCal3Bundler::iterator iter = collector_gtsamGeneralSFMFactorCal3Bundler.begin();
      iter != collector_gtsamGeneralSFMFactorCal3Bundler.end(); ) {
    delete *iter;
    collector_gtsamGeneralSFMFactorCal3Bundler.erase(iter++);
    anyDeleted = true;
  } }

  if(anyDeleted)
    cout <<
      "WARNING:  Wrap modules with variables in the workspace have been reloaded due to\n"
      "calling destructors, call 'clear all' again if you plan to now recompile a wrap\n"
      "module, so that your recompiled module is used instead of the old one." << endl;
  std::cout.rdbuf(outbuf);
}

void _special_cases_RTTIRegister() {
  const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_special_cases_rttiRegistry_created");
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
    if(mexPutVariable("global", "gtsam_special_cases_rttiRegistry_created", newAlreadyCreated) != 0) {
      mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
    }
    mxDestroyArray(newAlreadyCreated);
  }
}

void gtsamNonlinearFactorGraph_collectorInsertAndMakeBase_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef std::shared_ptr<gtsam::NonlinearFactorGraph> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamNonlinearFactorGraph.insert(self);
}

void gtsamNonlinearFactorGraph_deconstructor_1(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef std::shared_ptr<gtsam::NonlinearFactorGraph> Shared;
  checkArguments("delete_gtsamNonlinearFactorGraph",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamNonlinearFactorGraph::iterator item;
  item = collector_gtsamNonlinearFactorGraph.find(self);
  if(item != collector_gtsamNonlinearFactorGraph.end()) {
    collector_gtsamNonlinearFactorGraph.erase(item);
  }
  delete self;
}

void gtsamNonlinearFactorGraph_addPrior_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("addPriorPinholeCameraCal3Bundler",nargout,nargin-1,3);
  auto obj = unwrap_shared_ptr<gtsam::NonlinearFactorGraph>(in[0], "ptr_gtsamNonlinearFactorGraph");
  size_t key = unwrap< size_t >(in[1]);
  gtsam::PinholeCamera<gtsam::Cal3Bundler>& prior = *unwrap_shared_ptr< gtsam::PinholeCamera<gtsam::Cal3Bundler> >(in[2], "ptr_gtsamPinholeCameraCal3Bundler");
  std::shared_ptr<gtsam::noiseModel::Base> noiseModel = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[3], "ptr_gtsamnoiseModelBase");
  obj->addPrior<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(key,prior,noiseModel);
}

void gtsamSfmTrack_collectorInsertAndMakeBase_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef std::shared_ptr<gtsam::SfmTrack> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamSfmTrack.insert(self);
}

void gtsamSfmTrack_deconstructor_4(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef std::shared_ptr<gtsam::SfmTrack> Shared;
  checkArguments("delete_gtsamSfmTrack",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamSfmTrack::iterator item;
  item = collector_gtsamSfmTrack.find(self);
  if(item != collector_gtsamSfmTrack.end()) {
    collector_gtsamSfmTrack.erase(item);
  }
  delete self;
}

void gtsamSfmTrack_get_measurements_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("measurements",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::SfmTrack>(in[0], "ptr_gtsamSfmTrack");
  out[0] = wrap_shared_ptr(std::make_shared<std::vector<std::pair<size_t,Point2>>>(obj->measurements),"std.vectorpairsize_tPoint2", false);
}

void gtsamSfmTrack_set_measurements_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("measurements",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::SfmTrack>(in[0], "ptr_gtsamSfmTrack");
  std::shared_ptr<std::vector<std::pair<size_t,Point2>>> measurements = unwrap_shared_ptr< std::vector<std::pair<size_t,Point2>> >(in[1], "ptr_stdvectorpairsize_tPoint2");
  obj->measurements = *measurements;
}

void gtsamPinholeCameraCal3Bundler_collectorInsertAndMakeBase_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef std::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Bundler>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamPinholeCameraCal3Bundler.insert(self);
}

void gtsamPinholeCameraCal3Bundler_deconstructor_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef std::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Bundler>> Shared;
  checkArguments("delete_gtsamPinholeCameraCal3Bundler",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamPinholeCameraCal3Bundler::iterator item;
  item = collector_gtsamPinholeCameraCal3Bundler.find(self);
  if(item != collector_gtsamPinholeCameraCal3Bundler.end()) {
    collector_gtsamPinholeCameraCal3Bundler.erase(item);
  }
  delete self;
}

void gtsamGeneralSFMFactorCal3Bundler_collectorInsertAndMakeBase_9(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef std::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamGeneralSFMFactorCal3Bundler.insert(self);
}

void gtsamGeneralSFMFactorCal3Bundler_deconstructor_10(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef std::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>> Shared;
  checkArguments("delete_gtsamGeneralSFMFactorCal3Bundler",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamGeneralSFMFactorCal3Bundler::iterator item;
  item = collector_gtsamGeneralSFMFactorCal3Bundler.find(self);
  if(item != collector_gtsamGeneralSFMFactorCal3Bundler.end()) {
    collector_gtsamGeneralSFMFactorCal3Bundler.erase(item);
  }
  delete self;
}

void gtsamGeneralSFMFactorCal3Bundler_get_verbosity_11(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("verbosity",nargout,nargin-1,0);
  auto obj = unwrap_shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>>(in[0], "ptr_gtsamGeneralSFMFactorCal3Bundler");
  out[0] = wrap_shared_ptr(std::make_shared<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity>(obj->verbosity),"gtsam.GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>.Verbosity", false);
}

void gtsamGeneralSFMFactorCal3Bundler_set_verbosity_12(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("verbosity",nargout,nargin-1,1);
  auto obj = unwrap_shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>>(in[0], "ptr_gtsamGeneralSFMFactorCal3Bundler");
  std::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity> verbosity = unwrap_shared_ptr< gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity >(in[1], "ptr_gtsamGeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>Verbosity");
  obj->verbosity = *verbosity;
}


void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mstream mout;
  std::streambuf *outbuf = std::cout.rdbuf(&mout);

  _special_cases_RTTIRegister();

  int id = unwrap<int>(in[0]);

  try {
    switch(id) {
    case 0:
      gtsamNonlinearFactorGraph_collectorInsertAndMakeBase_0(nargout, out, nargin-1, in+1);
      break;
    case 1:
      gtsamNonlinearFactorGraph_deconstructor_1(nargout, out, nargin-1, in+1);
      break;
    case 2:
      gtsamNonlinearFactorGraph_addPrior_2(nargout, out, nargin-1, in+1);
      break;
    case 3:
      gtsamSfmTrack_collectorInsertAndMakeBase_3(nargout, out, nargin-1, in+1);
      break;
    case 4:
      gtsamSfmTrack_deconstructor_4(nargout, out, nargin-1, in+1);
      break;
    case 5:
      gtsamSfmTrack_get_measurements_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      gtsamSfmTrack_set_measurements_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      gtsamPinholeCameraCal3Bundler_collectorInsertAndMakeBase_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      gtsamPinholeCameraCal3Bundler_deconstructor_8(nargout, out, nargin-1, in+1);
      break;
    case 9:
      gtsamGeneralSFMFactorCal3Bundler_collectorInsertAndMakeBase_9(nargout, out, nargin-1, in+1);
      break;
    case 10:
      gtsamGeneralSFMFactorCal3Bundler_deconstructor_10(nargout, out, nargin-1, in+1);
      break;
    case 11:
      gtsamGeneralSFMFactorCal3Bundler_get_verbosity_11(nargout, out, nargin-1, in+1);
      break;
    case 12:
      gtsamGeneralSFMFactorCal3Bundler_set_verbosity_12(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

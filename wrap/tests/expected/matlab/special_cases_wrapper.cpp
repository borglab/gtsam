#include <gtwrap/matlab.h>
#include <map>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>

#include <folder/path/to/Test.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <path/to/ns1.h>
#include <path/to/ns1/ClassB.h>
#include <path/to/ns2.h>
#include <path/to/ns2/ClassA.h>
#include <path/to/ns3.h>

typedef Fun<double> FunDouble;
typedef PrimitiveRef<double> PrimitiveRefDouble;
typedef MyVector<3> MyVector3;
typedef MyVector<12> MyVector12;
typedef MultipleTemplates<int, double> MultipleTemplatesIntDouble;
typedef MultipleTemplates<int, float> MultipleTemplatesIntFloat;
typedef MyFactor<gtsam::Pose2, gtsam::Matrix> MyFactorPosePoint2;
typedef MyTemplate<gtsam::Point2> MyTemplatePoint2;
typedef MyTemplate<gtsam::Matrix> MyTemplateMatrix;
typedef gtsam::PinholeCamera<gtsam::Cal3Bundler> PinholeCameraCal3Bundler;
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3> GeneralSFMFactorCal3Bundler;

BOOST_CLASS_EXPORT_GUID(gtsam::Point2, "gtsamPoint2");
BOOST_CLASS_EXPORT_GUID(gtsam::Point3, "gtsamPoint3");

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
typedef std::set<boost::shared_ptr<gtsam::Point2>*> Collector_gtsamPoint2;
static Collector_gtsamPoint2 collector_gtsamPoint2;
typedef std::set<boost::shared_ptr<gtsam::Point3>*> Collector_gtsamPoint3;
static Collector_gtsamPoint3 collector_gtsamPoint3;
typedef std::set<boost::shared_ptr<MyBase>*> Collector_MyBase;
static Collector_MyBase collector_MyBase;
typedef std::set<boost::shared_ptr<MyTemplatePoint2>*> Collector_MyTemplatePoint2;
static Collector_MyTemplatePoint2 collector_MyTemplatePoint2;
typedef std::set<boost::shared_ptr<MyTemplateMatrix>*> Collector_MyTemplateMatrix;
static Collector_MyTemplateMatrix collector_MyTemplateMatrix;
typedef std::set<boost::shared_ptr<ns1::ClassA>*> Collector_ns1ClassA;
static Collector_ns1ClassA collector_ns1ClassA;
typedef std::set<boost::shared_ptr<ns1::ClassB>*> Collector_ns1ClassB;
static Collector_ns1ClassB collector_ns1ClassB;
typedef std::set<boost::shared_ptr<ns2::ClassA>*> Collector_ns2ClassA;
static Collector_ns2ClassA collector_ns2ClassA;
typedef std::set<boost::shared_ptr<ns2::ns3::ClassB>*> Collector_ns2ns3ClassB;
static Collector_ns2ns3ClassB collector_ns2ns3ClassB;
typedef std::set<boost::shared_ptr<ns2::ClassC>*> Collector_ns2ClassC;
static Collector_ns2ClassC collector_ns2ClassC;
typedef std::set<boost::shared_ptr<ClassD>*> Collector_ClassD;
static Collector_ClassD collector_ClassD;
typedef std::set<boost::shared_ptr<gtsam::NonlinearFactorGraph>*> Collector_gtsamNonlinearFactorGraph;
static Collector_gtsamNonlinearFactorGraph collector_gtsamNonlinearFactorGraph;
typedef std::set<boost::shared_ptr<gtsam::SfmTrack>*> Collector_gtsamSfmTrack;
static Collector_gtsamSfmTrack collector_gtsamSfmTrack;
typedef std::set<boost::shared_ptr<PinholeCameraCal3Bundler>*> Collector_gtsamPinholeCameraCal3Bundler;
static Collector_gtsamPinholeCameraCal3Bundler collector_gtsamPinholeCameraCal3Bundler;
typedef std::set<boost::shared_ptr<GeneralSFMFactorCal3Bundler>*> Collector_gtsamGeneralSFMFactorCal3Bundler;
static Collector_gtsamGeneralSFMFactorCal3Bundler collector_gtsamGeneralSFMFactorCal3Bundler;

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
  { for(Collector_ns1ClassA::iterator iter = collector_ns1ClassA.begin();
      iter != collector_ns1ClassA.end(); ) {
    delete *iter;
    collector_ns1ClassA.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_ns1ClassB::iterator iter = collector_ns1ClassB.begin();
      iter != collector_ns1ClassB.end(); ) {
    delete *iter;
    collector_ns1ClassB.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_ns2ClassA::iterator iter = collector_ns2ClassA.begin();
      iter != collector_ns2ClassA.end(); ) {
    delete *iter;
    collector_ns2ClassA.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_ns2ns3ClassB::iterator iter = collector_ns2ns3ClassB.begin();
      iter != collector_ns2ns3ClassB.end(); ) {
    delete *iter;
    collector_ns2ns3ClassB.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_ns2ClassC::iterator iter = collector_ns2ClassC.begin();
      iter != collector_ns2ClassC.end(); ) {
    delete *iter;
    collector_ns2ClassC.erase(iter++);
    anyDeleted = true;
  } }
  { for(Collector_ClassD::iterator iter = collector_ClassD.begin();
      iter != collector_ClassD.end(); ) {
    delete *iter;
    collector_ClassD.erase(iter++);
    anyDeleted = true;
  } }
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

void gtsamNonlinearFactorGraph_collectorInsertAndMakeBase_0(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::NonlinearFactorGraph> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamNonlinearFactorGraph.insert(self);
}

void gtsamNonlinearFactorGraph_deconstructor_1(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::NonlinearFactorGraph> Shared;
  checkArguments("delete_gtsamNonlinearFactorGraph",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamNonlinearFactorGraph::iterator item;
  item = collector_gtsamNonlinearFactorGraph.find(self);
  if(item != collector_gtsamNonlinearFactorGraph.end()) {
    delete self;
    collector_gtsamNonlinearFactorGraph.erase(item);
  }
}

void gtsamNonlinearFactorGraph_addPrior_2(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  checkArguments("addPriorPinholeCameraCal3Bundler",nargout,nargin-1,3);
  auto obj = unwrap_shared_ptr<gtsam::NonlinearFactorGraph>(in[0], "ptr_gtsamNonlinearFactorGraph");
  size_t key = unwrap< size_t >(in[1]);
  gtsam::PinholeCamera<gtsam::Cal3Bundler>& prior = *unwrap_shared_ptr< gtsam::PinholeCamera<gtsam::Cal3Bundler> >(in[2], "ptr_gtsamPinholeCameraCal3Bundler");
  boost::shared_ptr<gtsam::noiseModel::Base> noiseModel = unwrap_shared_ptr< gtsam::noiseModel::Base >(in[3], "ptr_gtsamnoiseModelBase");
  obj->addPrior<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(key,prior,noiseModel);
}

void gtsamSfmTrack_collectorInsertAndMakeBase_3(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::SfmTrack> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamSfmTrack.insert(self);
}

void gtsamSfmTrack_deconstructor_4(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::SfmTrack> Shared;
  checkArguments("delete_gtsamSfmTrack",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamSfmTrack::iterator item;
  item = collector_gtsamSfmTrack.find(self);
  if(item != collector_gtsamSfmTrack.end()) {
    delete self;
    collector_gtsamSfmTrack.erase(item);
  }
}

void gtsamPinholeCameraCal3Bundler_collectorInsertAndMakeBase_5(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Bundler>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamPinholeCameraCal3Bundler.insert(self);
}

void gtsamPinholeCameraCal3Bundler_deconstructor_6(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Bundler>> Shared;
  checkArguments("delete_gtsamPinholeCameraCal3Bundler",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamPinholeCameraCal3Bundler::iterator item;
  item = collector_gtsamPinholeCameraCal3Bundler.find(self);
  if(item != collector_gtsamPinholeCameraCal3Bundler.end()) {
    delete self;
    collector_gtsamPinholeCameraCal3Bundler.erase(item);
  }
}

void gtsamGeneralSFMFactorCal3Bundler_collectorInsertAndMakeBase_7(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  mexAtExit(&_deleteAllObjects);
  typedef boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>> Shared;

  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
  collector_gtsamGeneralSFMFactorCal3Bundler.insert(self);
}

void gtsamGeneralSFMFactorCal3Bundler_deconstructor_8(int nargout, mxArray *out[], int nargin, const mxArray *in[])
{
  typedef boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>> Shared;
  checkArguments("delete_gtsamGeneralSFMFactorCal3Bundler",nargout,nargin,1);
  Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
  Collector_gtsamGeneralSFMFactorCal3Bundler::iterator item;
  item = collector_gtsamGeneralSFMFactorCal3Bundler.find(self);
  if(item != collector_gtsamGeneralSFMFactorCal3Bundler.end()) {
    delete self;
    collector_gtsamGeneralSFMFactorCal3Bundler.erase(item);
  }
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
      gtsamPinholeCameraCal3Bundler_collectorInsertAndMakeBase_5(nargout, out, nargin-1, in+1);
      break;
    case 6:
      gtsamPinholeCameraCal3Bundler_deconstructor_6(nargout, out, nargin-1, in+1);
      break;
    case 7:
      gtsamGeneralSFMFactorCal3Bundler_collectorInsertAndMakeBase_7(nargout, out, nargin-1, in+1);
      break;
    case 8:
      gtsamGeneralSFMFactorCal3Bundler_deconstructor_8(nargout, out, nargin-1, in+1);
      break;
    }
  } catch(const std::exception& e) {
    mexErrMsgTxt(("Exception from gtsam:\n" + std::string(e.what()) + "\n").c_str());
  }

  std::cout.rdbuf(outbuf);
}

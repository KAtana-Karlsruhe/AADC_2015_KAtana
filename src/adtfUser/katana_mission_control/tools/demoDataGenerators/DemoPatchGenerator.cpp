/**
 * Katana Demo Patch Filter.
 * $Author: philipp $
 *
 */
#include "stdafx.h"
#include "math.h"
#include <fstream>
#include "DemoPatchGenerator.h"


/// Create filter for random number generator
ADTF_FILTER_PLUGIN("KATANA Demo Patch Generator Filter", OID_ADTF_DEMO_PATCH_GENERATOR_FILTER, cDemoPatchFilter)

cDemoPatchFilter::cDemoPatchFilter(const tChar* __info) : adtf::cTimeTriggeredFilter(__info)
{
  m_numberOfPatches = -1;
  m_cycle = 0;
  m_points = nullptr;
  m_types = nullptr;
  m_thetas = nullptr;

  m_upper_left = katana::Point(23000, 5000);
  m_down_right = katana::Point(4000, -5000);

  m_enable_field_of_view = true;

  SetPropertyInt("Interval", 500000);   //500ms = 2Hz
  SetPropertyBool("Enable field of view", m_enable_field_of_view);

  SetPropertyStr("Configuration File For Patches","");
  SetPropertyBool("Configuration File For Patches" NSSUBPROP_FILENAME, tTrue);
  SetPropertyStr("Configuration File For Patches" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "CSV Files (*.csv)");
}

cDemoPatchFilter::~cDemoPatchFilter()
{
}

tResult cDemoPatchFilter::initializePatchArray()
{
  cFilename fileConfig = GetPropertyStr("Configuration File For Patches");
  if (fileConfig.IsEmpty()) {
    LOG_ERROR("DemoPatchGenerator: Configured configuration file not found");
    RETURN_ERROR(ERR_INVALID_FILE);
  }
  ADTF_GET_CONFIG_FILENAME(fileConfig);
  fileConfig = fileConfig.CreateAbsolutePath(".");

  if (cFileSystem::Exists(fileConfig)) {
    calculateNumberOfPatches(fileConfig);
    std::ifstream infile(fileConfig);
    tInt32 x,y, type;
    tFloat32 theta;
    tInt32 count = 0;
    m_points = new katana::Point [m_numberOfPatches];
    m_thetas = new tFloat32[m_numberOfPatches];
    m_types = new tInt32[m_numberOfPatches];
    while (infile >> x >> y >> theta >> type && count < m_numberOfPatches) {
      m_points[count] = katana::Point(x,y);
      m_thetas[count] = theta;
      m_types[count] = type;
      LOG_INFO(adtf_util::cString::Format("DemoPatchGenerator: initialized Patch no.: %d x %d y %d theta %f type %d",count,x,y,theta,type));
      count++;
    }
  } else {
    LOG_ERROR(adtf_util::cString::Format("Patch configuration file not found."));
    RETURN_ERROR(ERR_INVALID_FILE);
  }
  RETURN_NOERROR;
}

tResult cDemoPatchFilter::calculateNumberOfPatches(cFilename fileConfig) {
  std::ifstream file(fileConfig);
  tInt32 counter = 0;
  tInt32 x,y, type;
  tFloat32 theta;
  while (file >> x >> y >> theta >> type) {
    counter++;
  }
  m_numberOfPatches = counter;
  RETURN_NOERROR;
}

tResult cDemoPatchFilter::Init(tInitStage eStage, __exception)
{
  RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr));

  if (eStage == StageFirst) {
    CreateOutputPins(__exception_ptr);
  }
  else if (eStage == StageNormal) {
    tInt nGenerateRate = GetPropertyInt("Interval");
    m_numberDataValues = GetPropertyInt("Numer Data Values");
    m_enable_field_of_view = GetPropertyBool("Enable field of view");
    RETURN_IF_FAILED(initializePatchArray());
    RETURN_IF_FAILED(SetInterval(nGenerateRate));
  }
  else if (eStage == StageGraphReady){
  }

  RETURN_NOERROR;
}

tResult cDemoPatchFilter::Shutdown(tInitStage eStage, __exception)
{
  if (eStage == StageGraphReady) {
  }
  else if (eStage == StageNormal) {
    if(m_points != nullptr){
      delete[] m_points;
    }
    m_points  = nullptr;

    if(m_thetas != nullptr) {
      delete[] m_thetas;
    }
    m_thetas = nullptr;

    if(m_types != nullptr) {
      delete[] m_types;
    }
    m_types = nullptr;
  }
  else if (eStage == StageFirst) {
  }

  return cTimeTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult cDemoPatchFilter::CreateOutputPins(__exception)
{
  // patch output
  RETURN_IF_FAILED(m_oPinOutput.Create("demo_patches", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_oPinOutput));

  // create pose input pin without media description
  RETURN_IF_FAILED(m_ipin_pose.Create("pose_buffer" , new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_pose));

/*
  // get pose media type for the pose output pin
  strDescSignalValue = pDescManager->GetMediaDescription("tPose");
  RETURN_IF_POINTER_NULL(strDescSignalValue);
  cObjectPtr<IMediaType> pTypePoseValueOutput = new cMediaType(0, 0, 0, "tPose", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  RETURN_IF_FAILED(pTypePoseValueOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputPose));

  // create and register the output pin
  RETURN_IF_FAILED(m_opin_pose.Create("pose", pTypePoseValueOutput, this));
  RETURN_IF_FAILED(RegisterPin(&m_opin_pose));
*/
  RETURN_NOERROR;
}


tResult cDemoPatchFilter::Cycle(__exception)
{

  if(m_numberOfPatches == -1) {
   std::cout << "Points not correctly initialized";
   RETURN_ERROR(ERR_FILE_NOT_FOUND);
  }

//  if(m_cycle % 2 == 0) {
    // Create Mediasample

  std::vector<u_int32_t> patches;

  // choose patches in field of view
  if (m_enable_field_of_view)
  {
    for (int32_t i = 0; i < m_numberOfPatches; i++)
    {
      katana::Point veh_coord = m_pose.transformToVehicle(katana::Point(m_points[i].x(), m_points[i].y()));
      if(veh_coord.getX() < m_down_right.getX() || veh_coord.getX() > m_upper_left.getX()
         || veh_coord.getY() < m_down_right.getY() || veh_coord.getY() > m_upper_left.getY())
      {
        continue;
      }

      // patch pose with index i is in field of view:
      patches.push_back(i);
    }


  }
  else
  {
    patches.push_back(m_cycle % m_numberOfPatches);
    patches.push_back((m_cycle+1) % m_numberOfPatches);
    patches.push_back((m_cycle+2) % m_numberOfPatches);
    patches.push_back((m_cycle+3) % m_numberOfPatches);
    m_cycle += 3;
  }

  // array for sending
  katana::sPatch s[patches.size()];

  for (size_t i = 0; i < patches.size(); i++)
  {
    u_int32_t num = patches[i];

    s[i].sp.x = m_points[num].x();
    s[i].sp.y = m_points[num].y();
    s[i].sp.theta = m_thetas[num];
    // leave ids out so mc can insert "virtual" patches for junctions
    s[i].id = num * 5;
    s[i].patch_type = m_types[num];
  }

  // alloc media sample and send
  // transmit number of patches and patches themselves
  const std::size_t number_size = sizeof(u_int8_t);
  u_int8_t number_of_patches = patches.size() < 256 ? (u_int8_t)patches.size() : 255;

  const std::size_t patchsize = patches.size() * sizeof(katana::sPatch);

  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
  RETURN_IF_FAILED(pMediaSample->AllocBuffer(patchsize + number_size));
  RETURN_IF_FAILED(pMediaSample->CopyBufferFrom(&number_of_patches, number_size, 0, 0));
  RETURN_IF_FAILED(pMediaSample->CopyBufferFrom(s, patchsize, 0, number_size));
  RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
  m_oPinOutput.Transmit(pMediaSample);


  m_cycle++;

  RETURN_NOERROR;
}

tResult cDemoPatchFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
  // first check what kind of event it is
  if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    // so we received a media sample, so this pointer better be valid.
    RETURN_IF_POINTER_NULL(pMediaSample);

    // by comparing it to our member pin variable we can find out which pin received
    // the sample
    if (pSource == &m_ipin_pose)
    {
      katana::sPose sp;
      pMediaSample->CopyBufferTo(&sp, sizeof(katana::sPose), 0, 0);
      m_pose.setX(sp.x);
      m_pose.setY(sp.y);
      m_pose.setTheta(sp.theta);
    }
  }

  RETURN_NOERROR;
}

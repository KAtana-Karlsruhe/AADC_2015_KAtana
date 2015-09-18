/**
 * Katana Demo Obstacle Filter.
 * $Author: philipp $
 *
 */
#include "stdafx.h"
#include "math.h"
#include <fstream>
#include "DemoObstacleGenerator.h"
#include "PinConversions.h"


/// Create filter for random number generator
ADTF_FILTER_PLUGIN("KATANA Demo Obstacle Generator Filter", OID_ADTF_DEMO_OBSTACLE_GENERATOR_FILTER, cDemoObstacleFilter)

cDemoObstacleFilter::cDemoObstacleFilter(const tChar* __info) : adtf::cTimeTriggeredFilter(__info)
{
  m_numberOfObstacles = -1;
  m_cycle = 0;
  m_poses = nullptr;
  m_x_dimensions = nullptr;
  m_y_dimensions = nullptr;
   
  m_upper_left = katana::Point(23000, 5000);
  m_down_right = katana::Point(4000, -5000);

  m_enable_field_of_view = true;

  SetPropertyInt("Interval", 1000000);   //200ms = 1Hz
  SetPropertyBool("Enable field of view", m_enable_field_of_view);
    
  SetPropertyStr("Configuration File For Obstacles",""); 
  SetPropertyBool("Configuration File For Obstacles" NSSUBPROP_FILENAME, tTrue); 
  SetPropertyStr("Configuration File For Obstacles" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "CSV Files (*.csv)"); 
}

cDemoObstacleFilter::~cDemoObstacleFilter()
{
}

tResult cDemoObstacleFilter::initializeObstacleArray()
{
  cFilename fileConfig = GetPropertyStr("Configuration File For Obstacles");
  if (fileConfig.IsEmpty()) {
    LOG_ERROR("DemoObstacleGenerator: Configured configuration file not found");
    RETURN_ERROR(ERR_INVALID_FILE);
  }	
  ADTF_GET_CONFIG_FILENAME(fileConfig);
  fileConfig = fileConfig.CreateAbsolutePath(".");

  if (cFileSystem::Exists(fileConfig)) {
    calculateNumberOfObstacles(fileConfig);
    std::ifstream infile(fileConfig);
    tInt32 x,y, x_dimension, y_dimension;
    tFloat32 theta;
    tInt32 count = 0;
    m_poses = new katana::Pose [m_numberOfObstacles];
    m_x_dimensions = new tInt32[m_numberOfObstacles];
    m_y_dimensions = new tInt32[m_numberOfObstacles];
    while (infile >> x >> y >> theta >> x_dimension >> y_dimension && count < m_numberOfObstacles) {
      m_poses[count] = katana::Pose(x,y,theta);
      m_x_dimensions[count] = x_dimension;
      m_y_dimensions[count] = y_dimension;
      LOG_INFO(adtf_util::cString::Format("DemoObstacleGenerator: initialized Obstacle no.: %d x (dim: %d) %d y (dim: %d) ",count,x,y,x_dimension,y_dimension));
      count++;
    }
  } else {
    LOG_ERROR(adtf_util::cString::Format("Obstacle configuration file not found."));
    RETURN_ERROR(ERR_INVALID_FILE);
  }
  RETURN_NOERROR;
}

tResult cDemoObstacleFilter::calculateNumberOfObstacles(cFilename fileConfig) {
  std::ifstream file(fileConfig);
  tInt32 counter = 0;
  tInt32 x,y, x_dimension, y_dimension;
  tFloat32 theta;
  while (file >> x >> y >> theta >> x_dimension >> y_dimension) {
    counter++;     
  }
  m_numberOfObstacles = counter;
  RETURN_NOERROR;
}

tResult cDemoObstacleFilter::Init(tInitStage eStage, __exception)
{
  RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr));
  
  if (eStage == StageFirst) {
    CreateOutputPins(__exception_ptr);
  }
  else if (eStage == StageNormal) {
    tInt nGenerateRate = GetPropertyInt("Interval");
    m_numberDataValues = GetPropertyInt("Numer Data Values");
    m_enable_field_of_view = GetPropertyBool("Enable field of view");
    RETURN_IF_FAILED(initializeObstacleArray());   
    RETURN_IF_FAILED(SetInterval(nGenerateRate));
  }
  else if (eStage == StageGraphReady){
  }
  
  RETURN_NOERROR;
}

tResult cDemoObstacleFilter::Shutdown(tInitStage eStage, __exception)
{
  if (eStage == StageGraphReady) {
  }
  else if (eStage == StageNormal) {
    if(m_poses != nullptr){
      delete[] m_poses;
    }
    m_poses  = nullptr;
    
    if(m_x_dimensions != nullptr) {
      delete[] m_x_dimensions;
    }
    m_x_dimensions = nullptr;
    
    if(m_y_dimensions != nullptr) {
      delete[] m_y_dimensions;
    }
    m_y_dimensions = nullptr;
  }
  else if (eStage == StageFirst) {
  }
  
  return cTimeTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult cDemoObstacleFilter::CreateOutputPins(__exception)
{
  // create and register the output pin
  RETURN_IF_FAILED(m_opin_obstacles.Create("obstacles" , new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_opin_obstacles));

  // create pose input pin without media description
  RETURN_IF_FAILED(m_ipin_pose.Create("pose_buffer" , new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_pose));

  RETURN_NOERROR;
}


tResult cDemoObstacleFilter::Cycle(__exception)
{
  
  if(m_numberOfObstacles == -1) {
   std::cout << "Obstacles not correctly initialized";
   RETURN_ERROR(ERR_FILE_NOT_FOUND); 
  }
  
//  if(m_cycle % 2 == 0) {
    // Create Mediasample

  std::vector<u_int32_t> obstacles;

  // choose patches in field of view
  if (m_enable_field_of_view)
  {
    for (int32_t i = 0; i < m_numberOfObstacles; i++)
    {
      katana::Point veh_coord = m_pose.transformToVehicle(katana::Point(m_poses[i].x(), m_poses[i].y()));
      if(veh_coord.getX() < m_down_right.getX() || veh_coord.getX() > m_upper_left.getX()
         || veh_coord.getY() < m_down_right.getY() || veh_coord.getY() > m_upper_left.getY())
      {
        continue;
      }

      // patch pose with index i is in field of view:
      obstacles.push_back(i);
    }


  }
  else
  {
    obstacles.push_back(m_cycle % m_numberOfObstacles);
  }

  for (u_int32_t num : obstacles)
  {   
    // Create sObstacleArray
    katana::sObstacle sobs[1];
    sobs[0].sp.x = m_poses[num].getX();
    sobs[0].sp.y = m_poses[num].getY();
    sobs[0].sp.theta = m_poses[num].getTheta();

    sobs[0].bounding_x = m_x_dimensions[num];
    sobs[0].bounding_y = m_x_dimensions[num];
    sobs[0].stamp = _clock->GetStreamTime();
    sobs[0].source = katana::ObstacleSource::IR_FRONT_CENTER_SHORT;

    // Create and send MediaSample
    size_t num_bytes = sizeof(katana::sObstacle);

    cObjectPtr<IMediaSample> media_sample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&media_sample));
    RETURN_IF_FAILED(media_sample->AllocBuffer(num_bytes));
    RETURN_IF_FAILED(media_sample->CopyBufferFrom(sobs, num_bytes, 0, 0));
    media_sample->SetTime(_clock->GetStreamTime());
    m_opin_obstacles.Transmit(media_sample);
  }

  m_cycle++;
    
  RETURN_NOERROR;
}

tResult cDemoObstacleFilter::OnPinEvent(IPin* pSource,
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

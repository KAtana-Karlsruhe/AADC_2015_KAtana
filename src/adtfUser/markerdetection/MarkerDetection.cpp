#include "stdafx.h"
#include "MarkerDetection.h"

#include "oadrive_vision/DrawUtil.h"


ADTF_FILTER_PLUGIN("Demo Filter", OID_ADTF_MARKER_DETECTION, cMarkerDetection)

cMarkerDetection::cMarkerDetection(const tChar* __info) : cAsyncDataTriggeredFilter(__info)
{
  SetPropertyStr("Dictionary File For Markers","");
  SetPropertyBool("Dictionary File For Markers" NSSUBPROP_FILENAME, tTrue);
  SetPropertyStr("Dictionary File For Markers" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)");

  // Sign filtering properties
  SetPropertyFloat("MIN_SIGN_AREA", 700.0f);
  SetPropertyFloat("MAX_SIGN_AREA", 1500.0f);
  SetPropertyFloat("MIN_SIGN_ASPECT_RATIO", 0.8f);
  SetPropertyFloat("MAX_SIGN_ASPECT_RATIO", 1.2f);
  SetPropertyFloat("MIN_PARKING_SIGN_AREA", 500.0f);
  SetPropertyFloat("MAX_PARKING_SIGN_AREA", 2700.0f);


  m_counter = 0;
  m_last_marker_set = false;
}

cMarkerDetection::~cMarkerDetection()
{
}

tResult cMarkerDetection::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
      // Create video input Pin
      RETURN_IF_FAILED(m_oPinInputVideo.Create("Video_RGB_input",IPin::PD_Input, static_cast<IPinEventSink*>(this)));
      RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));

      // pose input
      RETURN_IF_FAILED(m_ipin_pose.Create("pose_input", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
      RETURN_IF_FAILED(RegisterPin(&m_ipin_pose));

      // road sign output pin
      RETURN_IF_FAILED(m_oPinRoadSign.Create("road_signs" , new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
      RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSign));

    }
    else if (eStage == StageNormal)
    {
      loadDictionary();
      const double markerSize = 12 * 0.2 * 0.01;
      m_roadSignDetector.initializeWithoutCalibration(markerSize, m_dictionary);

      // initialize member properties
      m_min_sign_area = GetPropertyFloat("MIN_SIGN_AREA");
      m_max_sign_area = GetPropertyFloat("MAX_SIGN_AREA");
      m_min_sign_aspect_ratio = GetPropertyFloat("MIN_SIGN_ASPECT_RATIO");
      m_max_sign_aspect_ratio = GetPropertyFloat("MAX_SIGN_ASPECT_RATIO");
      m_min_parking_sign_area = GetPropertyFloat("MIN_PARKING_SIGN_AREA");
      m_max_parking_sign_area = GetPropertyFloat("MAX_PARKING_SIGN_AREA");
    }
    RETURN_NOERROR;
}

void cMarkerDetection::averageMarkers(oadrive::markers::RoadSignsContainer detectedRoadSigns)
{
  // filter invalid signs
  oadrive::markers::RoadSignsContainer filteredSigns = filterInvalidSigns(detectedRoadSigns);

  // proceed as usual on filtered signs
  m_marker_average.addFrame(filteredSigns);

  int best_id;
  if (!m_marker_average.checkFramesForMostSeen(best_id)) {

    // send disappeared Markers
    if(m_last_marker_set) {
      sendMarkers(m_last_marker, false);
    }
    m_last_marker_set = false;

    return;
  }

  for (const aruco::Marker& m : filteredSigns)
  {
    if (m.id == best_id)
    {
      // send new detected Markers
      sendMarkers(m, true);

      // send disappeared Markers
      if(m_last_marker_set && m.id != m_last_marker.id) {
	sendMarkers(m, false);
      }

      m_last_marker = m;
      m_last_marker_set = true;

      return;
    }
  }

  assert(false && "Currently seen ID disappeared, there's a bug");
}

oadrive::markers::RoadSignsContainer cMarkerDetection::filterInvalidSigns(oadrive::markers::RoadSignsContainer& roadSigns)
{
  oadrive::markers::RoadSignsContainer filtered;
  filtered.reserve(roadSigns.size());
  for (const aruco::Marker m : roadSigns)
  {
      katana::TrafficSign trafficSign = getSignFromMarker(m);
    // ignore signs:

      // if not a parking sign and too small or too big
      if (trafficSign != katana::TrafficSign::PARKING_AHEAD &&
          (m.getArea() < m_min_sign_area
           || m.getArea() > m_max_sign_area)) continue;

      // if unknown sign
      if (trafficSign == katana::TrafficSign::UNKNOWN) continue;

      // if parking sign and area too small or too big
      if (trafficSign == katana::TrafficSign::PARKING_AHEAD &&
          (m.getArea() < m_min_parking_sign_area
           || m.getArea() > m_max_parking_sign_area)) continue;

      // if marker is upside down
      if (oadrive::markers::RoadSignDetector::isUpsideDown(m, katana::IS_UPSIDE_DOWN_MARGIN)) continue;

      // if aspect ratio of marker is too small or too big
  #ifdef KATANA_MD_ENABLE_ASPECT_FILTERING
      float aspect = oadrive::markers::RoadSignDetector::calculateAspectRatio(m);
      if (aspect < m_min_sign_aspect_ratio || aspect > m_max_sign_aspect_ratio) continue;
  #endif

    // push back marker if no filter criterion holds true
    filtered.push_back(m);
  }
  return filtered;
}

void cMarkerDetection::sendMarkers(const aruco::Marker& marker, bool appearedMarker)
{
  vector<katana::sRoadSign> roadSigns;

  katana::sRoadSign s;
  s.sign = getSignFromMarker(marker);
  // set Pose of Car, from where sign was detected
  s.sp = m_vehicle_pose;
  s.size = marker.getArea();
  s.signFound = appearedMarker;
  s.yaw = oadrive::markers::RoadSignDetector::calculateAspectRatio(marker);
  s.isUpsideDown = oadrive::markers::RoadSignDetector::isUpsideDown(marker, katana::IS_UPSIDE_DOWN_MARGIN);
  roadSigns.push_back(s);

  sendRoadSignStruct(roadSigns);
}

tResult cMarkerDetection::sendRoadSignStruct(const vector<katana::sRoadSign>& roadSigns)
{
  //Don't send empty arrays
  if(roadSigns.size() == 0) RETURN_NOERROR;

  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

  std::size_t size = roadSigns.size() * sizeof(katana::sRoadSign);
  pMediaSample->AllocBuffer(size);
  pMediaSample->CopyBufferFrom(roadSigns.data(), size, 0, 0);
  pMediaSample->SetTime(_clock->GetStreamTime());

  m_oPinRoadSign.Transmit(pMediaSample);

#ifdef KATANA_ROAD_SIGN_DEBUG
    std::cout << "Transmitting " << roadSigns.size() << " roadSigns." << std::endl;
#endif

  RETURN_NOERROR;
}


tResult cMarkerDetection::OnAsyncPinEvent(IPin* pSource,
                                tInt nEventCode,
                                tInt nParam1,
                                tInt nParam2,
                                IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oPinInputVideo)
        {
    if(m_counter % katana::ROAD_SIGN_SKIP == 0)
    {
	    ProcessInput(pMediaSample);
	  }
	  ++m_counter;
        }
        // Update vehicle pose member
	else if (pSource == &m_ipin_pose)
	{
	  RETURN_IF_POINTER_NULL(pMediaSample);
	  // check if media sample has correct size
	  if(sizeof(katana::sPose) != pMediaSample->GetSize()) {
	    std::cout << "Warning: sizeof(katana::sPose) != pIMediaSample->GetSize()!" << std::endl;
	    RETURN_NOERROR;
	  }

	  // save current vehicle pose
	  katana::sPose read_pose;
	  pMediaSample->CopyBufferTo(&read_pose, sizeof(katana::sPose), 0, 0);

	  m_vehicle_pose = read_pose.toPose2d();
	} // end vehicle pose pin
        else
        {
            RETURN_ERROR(ERR_NOT_SUPPORTED);
        }
    }
    else if(nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {
      if (pSource == &m_oPinInputVideo)
      {
	 m_sInputFormat = (*m_oPinInputVideo.GetFormat());
      }
    }
    RETURN_NOERROR;
}

tResult cMarkerDetection::ProcessInput(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    if (pSample != NULL)
    {
      const tVoid* l_pSrcBuffer;
      if (IS_OK(pSample->Lock(&l_pSrcBuffer))) {
          cv::Mat inputImage  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);
          pSample->Unlock(l_pSrcBuffer);
          oadrive::markers::RoadSignsContainer detectedRoadSigns = m_roadSignDetector.detectRoadSigns(inputImage);
#ifdef KATANA_ROAD_SIGN_DEBUG
          std::cout << "Found " << detectedRoadSigns.size() << " markers" << std::endl;
#endif

          averageMarkers(detectedRoadSigns);

#ifdef KATANA_MD_WRITE_MARKERS
          cv::Mat output;
          inputImage.copyTo(output);
          oadrive::markers::RoadSignDetector::drawRegionOfInterest(output);
          for (const aruco::Marker m : detectedRoadSigns)
            oadrive::markers::RoadSignDetector::drawMarker(output, m);
          int best_id;
          if (m_marker_average.checkFramesForMostSeen(best_id))
            DrawUtil::putTextAt(output, Point(20, 20), "Most relevant Sign: " + std::to_string(best_id), 3);
          DrawUtil::writeImageToFile(output, "_markers");
#endif
      }
      else {
	RETURN_ERROR(ERR_UNKNOWN);
      }
    }
    RETURN_NOERROR;
}

tResult cMarkerDetection::loadDictionary() {
  //Get path of configuration file
  cFilename fileConfig = GetPropertyStr("Dictionary File For Markers");

  ADTF_GET_CONFIG_FILENAME(fileConfig);
  fileConfig = fileConfig.CreateAbsolutePath(".");

  if (fileConfig.IsEmpty() || !(cFileSystem::Exists(fileConfig)))
  {
    LOG_ERROR("Dictionary File For Markers not found");
    RETURN_ERROR(ERR_FILE_NOT_FOUND);
  }


  if(m_dictionary.fromFile(string(fileConfig))==false) {
    LOG_ERROR("Dictionary File For Markers not found");
  }

  if(m_dictionary.size()==0) {
    LOG_ERROR("Dictionary File For Markers not found");
  }

  RETURN_NOERROR;
}

katana::TrafficSign cMarkerDetection::getSignFromMarker(Marker m)
{
  switch (m.id)
    {
    case 491:
	//Vorfahrt gewaehren
	return katana::TrafficSign::JUNCTION_GIVE_WAY;
    case 371:
	//Vorfahrt an naechster Kreuzung
	return katana::TrafficSign::JUNCTION_PRIORITY;
    case 140:
	//Halt! Vorfahrt gewaehren (Stop)
	return katana::TrafficSign::JUNCTION_STOP_GIVE_WAY;
    case 484:
	//Parken
	return katana::TrafficSign::PARKING_AHEAD;
    case 166:
	//Vorgeschriebene Fahrtrichtung geradeaus
	return katana::TrafficSign::PRESCRIBED_DIRECTION;
    case 466:
	//Kreuzung
	return katana::TrafficSign::JUNCTION_PRIORITY_FROM_RIGHT;
    case 376:
	//Fussgaengerueberweg
	return katana::TrafficSign::UNKNOWN;
    case 46:
	//Kreisverkehr
	return katana::TrafficSign::UNKNOWN;
    case 340:
	//Ueberholverbot
	return katana::TrafficSign::UNKNOWN;
    case 306:
	//Verbot der Einfahrt
	return katana::TrafficSign::UNKNOWN;
    case 82:
	//Einbahnstrasse
	return katana::TrafficSign::UNKNOWN;
    default:
	return katana::TrafficSign::UNKNOWN;

    }
}

void cMarkerDetection::MarkerAverage::addFrame(const oadrive::markers::RoadSignsContainer& markers_in_frame)
{
  // mark every marker in current frame/add new IDCounter for new markers
  for (const aruco::Marker& m : markers_in_frame)
  {
    IDCounterMap::iterator it = m_id_counter.find(m.id);

    if (it == m_id_counter.end())
      addNewIDCounter(m.id);
    else
      it->second.seen = true;
  }

  // call frame on every queue object
  for (IDCounterMap::iterator it = m_id_counter.begin(); it != m_id_counter.end(); it++)
  {
    it->second.frame();
  }
}

bool cMarkerDetection::MarkerAverage::checkFramesForMostSeen(int &marker_id)
{
  int most_seen_id = 0;
  u_int8_t max_counter = NEED_FRAMES - 1;

  for (IDCounterMap::iterator it = m_id_counter.begin(); it != m_id_counter.end(); )
  {
    // frame needs to be seen in the LAST frame and enough times in the past
    if (it->second.frames.back() && it->second.counter > max_counter)
    {
      most_seen_id = it->first;
      max_counter = it->second.counter;
      it++;
    }
    else if (it->second.counter == 0)   //< can delete id object
    {
      it = m_id_counter.erase(it);
    }
    else
      it++;
  }

  if (max_counter > NEED_FRAMES - 1)
  {
    marker_id = most_seen_id;
    return true;
  }
  return false;
}

#include "Lanetracker.h"
#include "locale.h"
#include <iostream>

ADTF_FILTER_PLUGIN("KATANA Lanetracker", OID_ADTF_LANETRACKER, Lanetracker)

using namespace cv;

Lanetracker::Lanetracker(const tChar* __info)
  : cAsyncDataTriggeredFilter(__info)
  , m_framesAlreadySkipped(0)
  , m_consistent_patches(nullptr)
  , m_consistent_patches_size(0)
  , m_perception_state(katana::PerceptionState::DO_NOTHING)
  , m_request_processing_of_next_image(false)
{
  std::setlocale(LC_ALL, "C");

  // IPM Calibration XML
  SetPropertyStr("IPM-Config","");
  SetPropertyBool("IPM-Config" NSSUBPROP_FILENAME, tTrue);
  SetPropertyStr("IPM-Config" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");


  // Threshold property
  SetPropertyInt("Binary-Threshold", 200);
  SetPropertyBool("Use binary threshold", true);
  SetPropertyBool("Use HSV threshold", false);
  SetPropertyBool("Use histogram threshold", false);

}

Lanetracker::~Lanetracker()
{
  if (m_consistent_patches)
  {
    delete[] m_consistent_patches;
  }

}

tResult Lanetracker::Init(tInitStage eStage, __exception)
{
  #ifdef KATANA_LT_DEBUG
    std::cout << "[Lanetracker]: init" << std::endl;
  #endif
  RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));

  if (eStage == StageFirst)
  {
    CreateOutputPins();
    CreateInputPins();
  }
  else if (eStage == StageNormal)
  {
    // IPM XML
    cFilename fileConfig = GetPropertyStr("IPM-Config");
    if (fileConfig.IsEmpty())
    {
      LOG_ERROR("IPM Configuration: IPM configuration file not found!");
      RETURN_ERROR(ERR_INVALID_FILE);
    }

    ADTF_GET_CONFIG_FILENAME(fileConfig);
    fileConfig = fileConfig.CreateAbsolutePath(".");
    if (!cFileSystem::Exists(fileConfig))
    {
      RETURN_ERROR(ERR_INVALID_FILE);
    }

    // Binary threshold
    u_int8_t binaryThreshold = GetPropertyInt("Binary-Threshold");
    bool useBinaryThreshold = GetPropertyBool("Use binary threshold");

    bool useHSVThreshold = GetPropertyBool("Use HSV threshold");

    bool useHistogramThreshold = GetPropertyBool("Use histogram threshold");

    Threshold thresholdEnum;

    if(useBinaryThreshold && !useHSVThreshold && !useHistogramThreshold)
    {
      thresholdEnum = Threshold::USE_BINARY_THRESHOLD;
    }else if(useHSVThreshold && !useBinaryThreshold && !useHistogramThreshold)
    {
      thresholdEnum = Threshold::USE_HSV_THRESHOLD;
    }else if(useHistogramThreshold && !useBinaryThreshold && !useHSVThreshold)
    {
      thresholdEnum = Threshold::USE_HISTOGRAM_THRESHOLD;
    }
    else
    {
      std::cout << "******************* Threshold Property of the Lanetracker filter are not set ******************" << std::endl;
      throw 1;
    }

    // create new worker thread
    m_oProcessingThread = std::make_shared<cImageProcessing>(std::bind(&Lanetracker::transmitPatches, this, std::placeholders::_1, std::placeholders::_2), binaryThreshold, thresholdEnum);

    if (!m_oProcessingThread->initialize(std::string(fileConfig)))
    {
      std::cout <<"Katana config is invalid." <<std::endl;
      RETURN_ERROR(ERR_INVALID_FILE);
    }


  }
  else if (eStage == StageGraphReady)
  {
    m_oProcessingThread->setReady(true);
  }

  RETURN_NOERROR;
}

tResult Lanetracker::Shutdown(tInitStage eStage, __exception)
{
    if (eStage == StageNormal)
    {
        m_oProcessingThread->WaitForExit(1000000);   // wait for worker thread to exit, timeout in microseconds
    }
    return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult Lanetracker::prepareProcessing()
{

  // lock accessing pose and consistent patches
  std::lock_guard<std::mutex> lock(m_pose_mutex);

  // create copy of current pose, belonging to the newest image (TODO: maybe keep queue of poses and check streamTime for better match?)
  m_oProcessingThread->pose() = m_vehicle_pose;

  /*
   * update perception state of current worker thread.
   */
  m_oProcessingThread->perceptionState() = m_perception_state;
  m_oProcessingThread->patchesToLookFor() = m_patches_to_look_for;
  m_oProcessingThread->numberOfStitches() = m_number_of_stitches;
  m_oProcessingThread->matchingThreshold() = m_matching_threshold;

  // Create copy of current consistent patches for worker thread
  m_oProcessingThread->consistendPatches().clear();
  if (m_consistent_patches_size > 0)
  {
    assert(m_consistent_patches != nullptr);
    for (size_t i = 0; i < m_consistent_patches_size; i++)
    {
      m_oProcessingThread->consistendPatches().push_back(m_consistent_patches[i]);
    }
  }

  // if we have a new image to work on, process it
  #ifdef KATANA_LT_NEXT_RUN
    std::cout << "Processing new image!" << std::endl;
  #endif

#ifdef KATANA_LT_MEASURE_PERFORMANCE
    ++m_performance_counter;
    const u_int8_t count_frames = 10;
    if (m_performance_counter % count_frames == 0)
    {
      const std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

      const double diff_secs = std::chrono::duration_cast<std::chrono::microseconds>(now - m_last_timestamp).count() * 0.000001;

      std::cout <<"LT PERFORMANCE: " <<(double)count_frames/diff_secs <<" Hz" <<std::endl;

      m_last_timestamp = now;
    }
#endif

  // start worker thread to process the latest image, this works on created copies of used data
  m_oProcessingThread->setImage(m_image);

#ifdef KATANA_LT_DEBUG
  std::cout << "[Lanetracker]: prepare processing: creating thread" <<std::endl;
#endif

  // start processing in thread
  RETURN_IF_FAILED(m_oProcessingThread->Create(cKernelThread::TF_Suspended))

  #ifdef KATANA_LT_DEBUG
    std::cout << "[Lanetracker]: prepare processing: starting thread" <<std::endl;
  #endif

  m_oProcessingThread->Run();

  RETURN_NOERROR;
}

tResult Lanetracker::OnAsyncPinEvent(IPin* pSource,
                                         tInt nEventCode,
                                         tInt nParam1,
                                         tInt nParam2,
                                         IMediaSample* pIMediaSample)
{
  // update image
  if (pSource == &m_oInputPin && nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    // skip first few frames to ensure camera image is valid
    if (m_framesAlreadySkipped >= FRAMES_TO_BE_SKIPPED && m_oProcessingThread->isReady() &&
        ( m_request_processing_of_next_image || m_perception_state == katana::PerceptionState::RECOVERING) )
    {
#ifdef KATANA_LT_DEBUG
      std::cout << "[Lanetracker]: capturing new image for processing" <<std::endl;
#endif

      // set thread to unready -> prevent from requesing another image
      m_oProcessingThread->setReady(false);
      m_request_processing_of_next_image = false;

      RETURN_IF_POINTER_NULL(pIMediaSample);
      // create the image (cv::Mat) out of the current media sample
      const tVoid* l_pSrcBuffer;
      if (IS_OK(pIMediaSample->Lock(&l_pSrcBuffer)))
      {
        IplImage* cv_image = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        cvSetData(cv_image, (char*)l_pSrcBuffer, cv_image->widthStep);

        // lock overwriting of thread class image
        m_pose_mutex.lock();
        m_image = cvarrToMat(cv_image, true);
        m_pose_mutex.unlock();

        cvReleaseImageHeader(&cv_image);
        pIMediaSample->Unlock(&l_pSrcBuffer);
      }
      else
      {
        // let us try this again
        m_oProcessingThread->setReady(true);
        m_request_processing_of_next_image = true;
        RETURN_ERROR(ERR_UNEXPECTED);
      }
      /********* WE ARE READY TO PROCESS **********/
      return prepareProcessing();
    }
    else
    {
      // skipped frame
      m_framesAlreadySkipped++;
    }
  } // end image input pin

  // Update video input format
  else if (pSource == &m_oInputPin && nEventCode == IPinEventSink::PE_MediaTypeChanged)
  {
    cObjectPtr<IMediaType> pType;
    RETURN_IF_FAILED(m_oInputPin.GetMediaType(&pType));

    cObjectPtr<IMediaTypeVideo> pTypeVideo;
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

    const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
    if (pFormat != NULL)
    {
      cMemoryBlock::MemCopy(&m_sInputFormat, pFormat, sizeof(tBitmapFormat));
    }
  } // end video input format

  // Update vehicle pose member
  else if (pSource == &m_ipin_pose && nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    RETURN_IF_POINTER_NULL(pIMediaSample);
    // check if media sample has correct size
    if(sizeof(katana::sPose) != pIMediaSample->GetSize()) {
      std::cout << "Warning: sizeof(katana::sPose) != pIMediaSample->GetSize()!" << std::endl;
      RETURN_NOERROR;
    }

    // save current vehicle pose
    katana::sPose read_pose;
    pIMediaSample->CopyBufferTo(&read_pose, sizeof(katana::sPose), 0, 0);

    // lock accessing pose
    std::lock_guard<std::mutex> lock(m_pose_mutex);
    m_vehicle_pose = read_pose.toPose2d();
  } // end vehicle pose pin

  // Update current consistent patches member and perception state and start thread if it is ready!
  else if (pSource == &m_ipin_patches && nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    RETURN_IF_POINTER_NULL(pIMediaSample);
    #ifdef KATANA_LT_DEBUG
      std::cout << "[Lanetracker]: filter thread received message from MC - " << m_vehicle_pose << std::endl;
    #endif

    // lock update of consistent patches
    std::lock_guard<std::mutex> lock(m_pose_mutex);


    // allocate new storage, fill it with the media sample and update patches size
    if (m_consistent_patches)
    {
      delete[] m_consistent_patches;
      m_consistent_patches = nullptr;
    }

    // first byte contains perception state
    const std::size_t mediaSampleSize = pIMediaSample->GetSize();
    assert(mediaSampleSize >= 1 && "Empty mediasample");

    static const std::size_t statusSize = sizeof(m_perception_state);
    static const std::size_t numberSize = sizeof(m_number_of_stitches);
    static const std::size_t patchesToLookForSize = sizeof(m_patches_to_look_for);
    static const std::size_t thresholdSize = sizeof(m_matching_threshold);
    static const std::size_t patchSize = sizeof(katana::sPatch);

    const std::size_t patch_array_size = mediaSampleSize - statusSize - numberSize - patchesToLookForSize - thresholdSize;

#ifndef NDEBUG
    assert(patch_array_size % patchSize == 0 && "Media Sample has wrong size");
#else
    if (patch_array_size % patchSize != 0)
      RETURN_NOERROR; //< silently drop instead of crashing
#endif


    m_consistent_patches_size = patch_array_size/patchSize;
    if (patch_array_size > 0)
    {
      m_consistent_patches = new katana::sPatch[m_consistent_patches_size];
    }

    // copy first <statusSize> bytes to state flag
    pIMediaSample->CopyBufferTo(&m_perception_state, statusSize, 0, 0);
    pIMediaSample->CopyBufferTo(&m_number_of_stitches, numberSize, statusSize, 0);
    pIMediaSample->CopyBufferTo(&m_patches_to_look_for, patchesToLookForSize, statusSize + numberSize, 0);
    pIMediaSample->CopyBufferTo(&m_matching_threshold, thresholdSize, statusSize + numberSize + patchesToLookForSize, 0);

    // copy last <mediaSampleSize - statusSize> bytes to consistent patches
    pIMediaSample->CopyBufferTo(m_consistent_patches, patch_array_size, statusSize + numberSize + patchesToLookForSize + thresholdSize, 0);


    #ifdef KATANA_LT_RECEIVE_PATCHES
      std::cout << "Receiving " << to_string(m_consistent_patches_size) << " new consistent patches!" << std::endl;
      for (unsigned int i = 0; i < m_consistent_patches_size; i++)
      {
        katana::sPatch patch = m_consistent_patches[i];
        std::cout << "MC in World: [" << std::to_string(patch.sp.x) << ", " << std::to_string(patch.sp.y) << ", " << std::to_string(patch.sp.theta) << "]" << std::endl;
      }
    #endif

  #ifndef USE_DEBUG_TRIGGER_PIN
    // if thread is ready and are told to do something, proceed with initialising the thread with latest data
    if (m_oProcessingThread->isReady())
    {
      if (m_perception_state != katana::PerceptionState::DO_NOTHING)
      {
        // maybe the thread is still on its way to exit
        m_oProcessingThread->WaitForExit();

        // set flag, let image pin process on arrival of new image
        m_request_processing_of_next_image = true;
      }
      else    //< answer immedialty with empty patches vector
      {
        transmitPatches(vector<katana::sPatch>(), katana::PerceptionState::DO_NOTHING);
      }
    }
    else
    {
      std::cout <<"********** WARNING **********: You called the lanetracker while still working, disregarding your command..." <<std::endl;
      assert(false && "********** WARNING **********: You called the lanetracker while still working, disregarding your command...");
    }
  #endif

  }// end consistent patch pin

#ifdef USE_DEBUG_TRIGGER_PIN
  else if(pSource == &m_iDebug && nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    // copy first byte to obtain debug code
    u_int8_t code;
    pIMediaSample->CopyBufferTo(&code, sizeof(u_int8_t), 0, 0);

    if (code == 0)
    {
      std::cout <<"EXTERN TRIGGERED: Request processing of next image." <<std::endl;
      m_request_processing_of_next_image = true;
    }
  }
#endif

  RETURN_NOERROR;
}

tResult Lanetracker::transmitPatches(const vector<katana::sPatch>& patches, katana::PerceptionState perception_state)
{
  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

  // transmit number of patches, perception state and patches
  //const std::size_t number_size = sizeof(u_int8_t);
  //u_int8_t number_of_patches = patches.size() < 256 ? (u_int8_t)patches.size() : 255;

  const std::size_t state_size = sizeof(katana::PerceptionState);

  const std::size_t patch_size = patches.size() * sizeof(katana::sPatch);

  pMediaSample->AllocBuffer(state_size + patch_size);
  pMediaSample->CopyBufferFrom(&perception_state, state_size, 0, 0);
  pMediaSample->CopyBufferFrom(patches.data(), patch_size, 0, state_size);
  pMediaSample->SetTime(_clock->GetStreamTime());

#ifdef KATANA_LT_TRANSMIT_PATCHES
    std::cout << "[Lanetracker] Transmitting " << patches.size() << " patches to MC. perception state: " << perception_state << std::endl;
#endif

  m_opin_patches.Transmit(pMediaSample);


  RETURN_NOERROR;
}

tResult Lanetracker::CreateOutputPins(__exception)
{
    RETURN_IF_FAILED(m_opin_patches.Create("mc_patches", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), NULL));
    RETURN_IF_FAILED(RegisterPin(&m_opin_patches));

    RETURN_NOERROR;
}

tResult Lanetracker::CreateInputPins(__exception)
{
  //! Input pins
  // video input
  RETURN_IF_FAILED(m_oInputPin.Create("input",IPin::PD_Input, static_cast<IPinEventSink*>(this)));
  RETURN_IF_FAILED(RegisterPin(&m_oInputPin));

  // patch input
  RETURN_IF_FAILED(m_ipin_patches.Create("consistent_patches", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), this));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_patches));

  // pose input
  RETURN_IF_FAILED(m_ipin_pose.Create("pose_input", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_pose));

  // debug pin
  RETURN_IF_FAILED(m_iDebug.Create("debug_input", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), this));
  RETURN_IF_FAILED(RegisterPin(&m_iDebug));

  RETURN_NOERROR;
}

vector<TrackSegment> Lanetracker::cImageProcessing::buildConsistentPatches()
{
  vector<TrackSegment> consistentPatches;

  // return empty vector on empty input
  if (m_consistent_patches_worker.empty())
    return consistentPatches;

#ifdef KATANA_LT_RECEIVE_PATCHES
  std::cout << "Received " << to_string(m_consistent_patches_worker.size()) << "patches!" << std::endl;
  std::cout << "[Lanetracker]: currentPose used to transform world to pixel: " << m_vehicle_pose_worker << std::endl;
#endif

  for (const katana::sPatch& patch : m_consistent_patches_worker)
  {
    Patch::PatchType type = (Patch::PatchType)patch.patch_type;

    // transform pose from world to pixel coordinates
    Pose2d vehicle = m_vehicle_pose_worker.inverse() * patch.sp.toPose2d();
    ManagedPose inPixel = ManagedPose::transformCarToPixel(ManagedPose(ExtendedPose2d(vehicle), ManagedPose::Context::IN_CAR));

#ifdef KATANA_LT_RECEIVE_PATCHES
    std::cout << "MC in Pixel: " + inPixel.toString() << std::endl;
#endif

    // build new patch at the given startjunction with correct rotation and ID
    float rotation = inPixel.getThetaInRad();
    TrackSegment track = PatchFactory::getFactory()->getTrackByType(type, rotation);
    Point baseStart(track.getStartJunction().getX(), track.getStartJunction().getY());
    Point targetStart(inPixel.getX(), inPixel.getY());
    track.translate(targetStart - baseStart);
    track.setID(patch.id);
    consistentPatches.push_back(track);
  }
  return consistentPatches;
}


void Lanetracker::cImageProcessing::process()
{
  // preprocess input and save in preprocessed.
  std::string suffix;
  Mat preprocessed;

  Mat birdView;
  m_imagePreprocessor->convertToBirdView(m_image_worker, birdView);

  #ifdef KATANA_LT_WRITE_INPUT
    DrawUtil::writeImageToFile(m_image_worker, "_image");
  #endif


  if (m_threshold_to_use == Threshold::USE_BINARY_THRESHOLD)
  {
    m_imagePreprocessor->doBinaryThreshhold(birdView, preprocessed);
    suffix = "_binary_";
  } else if (m_threshold_to_use == Threshold::USE_HISTOGRAM_THRESHOLD)
  {
#ifdef KATANA_LT_DEBUG
  std::cout << "[Lanetracker]: Using Histogram" << std::endl;
#endif
    //recalculate threshold
    if(m_framecount == 0) {
      float percentage;
      if(m_perception_state_worker == katana::PerceptionState::JUNCTION_AHEAD || m_perception_state_worker == katana::PerceptionState::JUNCTION_AHEAD_AGAIN) {
#ifdef KATANA_LT_DEBUG
  std::cout << "[Lanetracker]: now using percentage of JUNCTION_AHEAD" << std::endl;
#endif
          percentage = 0.075;
      } else {
          percentage = 0.06;
      }
      cv::Rect fieldOfView = ManagedPose::getFieldOfVisionRectanglePixel();
      cv::Point center(fieldOfView.x + fieldOfView.size().width/2, fieldOfView.y + fieldOfView.size().height/2);
      Mat matOfView;
      Size size(fieldOfView.width, fieldOfView.height);
      getRectSubPix(birdView, size, center, matOfView);
      int thresh = m_imagePreprocessor->estimateThreshold(matOfView, percentage);
      m_imagePreprocessor->setThresholdValue(thresh);
      #ifdef KATANA_LT_DEBUG
        std::cout << "[Lanetracker]: Changed to threshold " << thresh << std::endl;
      #endif
      #ifdef KATANA_LT_WRITE_PREPROCESSED
        DrawUtil::writeImageToFile(matOfView, suffix + std::to_string(m_imagePreprocessor->getThresholdValue()));
      #endif
    }

    m_imagePreprocessor->doBinaryThreshhold(birdView, preprocessed);
    suffix = "_histogram_";
  }
  else if(m_threshold_to_use == Threshold::USE_HSV_THRESHOLD)
  {
    m_imagePreprocessor->doHSVThreshold(birdView, preprocessed, false);
    suffix = "_HSV_";
#ifdef KATANA_LT_WRITE_PREPROCESSED
  DrawUtil::writeImageToFile(birdView, "_calibration_");
#endif
  }
  #ifdef KATANA_LT_WRITE_PREPROCESSED
    DrawUtil::writeImageToFile(preprocessed, suffix + std::to_string(m_imagePreprocessor->getThresholdValue()));
  #endif

  #ifdef KATANA_LT_DEBUG
    std::cout << "[Lanetracker]: Stitcher starting with perception state " << m_perception_state_worker << std::endl;
  #endif

  assert(m_perception_state_worker != katana::PerceptionState::DO_NOTHING && "[Lanetracker] Perception state is DO_NOTHING but process() was called!");

  // create vector with known patches from mission control (will be empty
  vector<TrackSegment> consistend_tracks = buildConsistentPatches();

  // obtain STITCHPROFILE
  oadrive::vision::PatchSelector::STITCHPROFILE profile;
  switch(m_perception_state_worker)
  {
  case katana::PerceptionState::DETERMINING_PARKING_SPOT:
  {
    #ifdef KATANA_LT_DEBUG
      std::cout << "[Lanetracker]: Call STITCH_PARKING_PULL_OUT" << std::endl;
    #endif
    profile = oadrive::vision::PatchSelector::STITCH_PARKING_PULL_OUT;
    break;
  }
  case katana::PerceptionState::INITIALIZE:
  {
    #ifdef KATANA_LT_DEBUG
      std::cout << "[Lanetracker]: Call STITCH_INITIAL" << std::endl;
    #endif
    profile = oadrive::vision::PatchSelector::STITCH_INITIAL;
    break;
  }
  case katana::PerceptionState::RECOVERING:
  {
    #ifdef KATANA_LT_DEBUG
      std::cout << "[Lanetracker]: Call STITCH_EMERGENCY" << std::endl;
    #endif
    profile = oadrive::vision::PatchSelector::STITCH_EMERGENCY;
    break;
  }
  case katana::PerceptionState::NORMAL:
  {
    #ifdef KATANA_LT_DEBUG
      std::cout << "[Lanetracker]: Call STITCH_NORMAL" << std::endl;
    #endif
    profile = oadrive::vision::PatchSelector::STITCH_NORMAL;
    break;
  }
  case katana::PerceptionState::JUNCTION_AHEAD:
  {
    #ifdef KATANA_LT_DEBUG
      std::cout << "[Lanetracker]: Call STITCH_JUNCTION" << std::endl;
    #endif
    profile = oadrive::vision::PatchSelector::STITCH_JUNCTION;
    break;
  }
  case katana::PerceptionState::JUNCTION_AHEAD_AGAIN:
  {
    #ifdef KATANA_LT_DEBUG
      std::cout << "[Lanetracker]: Call STITCH_JUNCTION_AGAIN" << std::endl;
    #endif
    profile = oadrive::vision::PatchSelector::STITCH_JUNCTION_AGAIN;
    break;
  }
  case katana::PerceptionState::AFTER_JUNCTION:
  {
    #ifdef KATANA_LT_DEBUG
      std::cout << "[Lanetracker]: Call STITCH_AFTER_JUNCTION" << std::endl;
    #endif
    profile = oadrive::vision::PatchSelector::STITCH_AFTER_JUNCTION;
    break;
  }

  default:
    assert(false && "Unknown katana::PerceptionState");
  }

  m_stitcher.updatePose(m_vehicle_pose_worker);

  // call stitcher
  vector<TrackSegment> tracks = m_stitcher.stitch(preprocessed, consistend_tracks, profile, m_number_of_stitches_worker, m_patches_to_look_for_worker, m_matching_threshold_worker);

  #ifdef KATANA_LT_DEBUG
    std::cout << "[Lanetracker]: Stitcher finished" << std::endl;
  #endif

#ifdef KATANA_LT_TRANSMIT_PATCHES
  std::cout << "[Lanetracker]: currentPose, used to transform pixel to world: " << m_vehicle_pose_worker << std::endl;
#endif

  // create patches vector
  std::vector<katana::sPatch> patches;
  for (const TrackSegment& t : tracks)
  {
    // transform to world coordinates
    ManagedPose vehicle_coord = ManagedPose::transformPixelToCar(t.getStartJunction());
    Pose2d world_coord = m_vehicle_pose_worker * vehicle_coord.getPose().getPose();

    katana::sPatch sp;
    sp.patch_type = (int32_t)t.getPatchType();
    sp.id = t.getID();
    sp.sp.fromPose(world_coord);
    sp.match_value = t.getMatchingValue();

    #ifdef KATANA_LT_TRANSMIT_PATCHES
      std::cout << "Start in Pixel: " << t.getStartJunction().toString() << endl;
      std::cout << "Start in world: [" << std::to_string(sp.sp.x) << ", " << std::to_string(sp.sp.y) << ", " << std::to_string(sp.sp.theta) << "]" << std::endl;
    #endif

    patches.push_back(sp);
  }

  // Set this thread to ready before transmitting -> make sure that it is ready when asynchronous pin->newPatches gets called via mission control
  m_ready = true;

  m_framecount++;
  m_framecount %= 15;
  // CHECH FOR EMPTY TRACK/PATCHES AND SEND INFO BYTE TO MISSION CONTROL
  // transmit track information resulting from the stitching
  m_transmit_func(patches, m_perception_state_worker);
}

/**
 * The thread function which will be called in a loop.
 * @return error code to exit the loop after on processing
 */
tResult Lanetracker::cImageProcessing::ThreadFunc()
{
#ifdef KATANA_LT_DEBUG
    std::cout << "[Lanetracker]: Process start" << std::endl;
#endif
    process();
#ifdef KATANA_LT_DEBUG
    std::cout << "[Lanetracker]: Process end" << std::endl;
#endif
    // after image is processed, return an error code to let this thread end and set state to TS_FINISHED
    return ERR_CANCELLED;
}

Lanetracker::cImageProcessing::cImageProcessing(Lanetracker::cImageProcessing::PatchesTransmitFunc transmit_func, u_int8_t binaryThreshold, Threshold thresholdToUse)
  : cKernelThread()
  , m_ready(false)
  , m_transmit_func(transmit_func)
  , m_threshold_to_use(thresholdToUse)
//  , m_use_binary_threshold(useBinaryThreshold)
{
  m_imagePreprocessor = new ImagePreprocessor(binaryThreshold);
  m_framecount = 0;
}

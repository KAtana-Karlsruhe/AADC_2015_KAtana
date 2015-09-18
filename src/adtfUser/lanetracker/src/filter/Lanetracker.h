#ifndef _LANETRACKER_HEADER_
#define _LANETRACKER_HEADER_

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <mutex>

#ifdef KATANA_LT_MEASURE_PERFORMANCE
#include <chrono>
#endif

#include <oadrive_vision/Stitcher.h>
#include <oadrive_vision/TrackSegment.h>
#include <oadrive_vision/ImagePreprocessor.h>
#include <oadrive_vision/DrawUtil.h>

#include "katanaCommon/katanaCommon.h"
#include "PinConversions.h"

using namespace adtf;

//#define USE_DEBUG_TRIGGER_PIN

#define OID_ADTF_LANETRACKER "adtf.aadc.lanetracker"

enum class Threshold : u_int8_t {
  USE_BINARY_THRESHOLD=0,
  USE_HSV_THRESHOLD=1,
  USE_HISTOGRAM_THRESHOLD=2,
  TOTAL_THRESHOLD=3
};


class Lanetracker : public cAsyncDataTriggeredFilter
{
    ADTF_FILTER(OID_ADTF_LANETRACKER, "KATANA Lanetracker", OBJCAT_Converter)

public:

  Lanetracker(const tChar* __info);

  virtual ~Lanetracker();

  tResult CreateOutputPins(__exception = NULL);
  tResult CreateInputPins(__exception = NULL);
  tResult Init(tInitStage eStage, __exception = NULL);
  tResult Shutdown(tInitStage eStage, __exception = NULL);
  tResult OnAsyncPinEvent(IPin* pSource,
                           tInt nEventCode,
                           tInt nParam1,
                           tInt nParam2,
                           IMediaSample* pIMediaSamples);


private:

  /**
   * @brief The cImageProcessing class
   * Manages the processing of the image set to m_image
   * in an extra thread
   */
  class cImageProcessing : public cKernelThread
  {
  public:
    typedef std::function<tResult(const vector<katana::sPatch>& patches, katana::PerceptionState perception_state)> PatchesTransmitFunc;

    //! Constructor
    cImageProcessing() = delete;
    cImageProcessing(PatchesTransmitFunc transmit_func, u_int8_t thresholdValue, Threshold treshold);


    //! Destructor
    virtual ~cImageProcessing()     { delete m_imagePreprocessor; }

    //! Load config for IPM and Pixel coordinate transformation
    bool initialize(const std::string& filename)
    {


      // initializing (order is important: first imagePreprocessor including managedPose, then Stitcher including PatchFactory)
      return m_imagePreprocessor->initialize(filename) && m_stitcher.initializePatches();
    }

    //! data access
    oadrive::core::Pose2d& pose()                         { return m_vehicle_pose_worker; }
    std::vector<katana::sPatch>& consistendPatches()      { return m_consistent_patches_worker; }
    cv::Mat& image()                                      { return m_image_worker; }
    void setImage(cv::Mat& image)                         { image.copyTo(m_image_worker); }
    katana::PerceptionState& perceptionState()            { return m_perception_state_worker; }
    u_int8_t& numberOfStitches()                          { return m_number_of_stitches_worker; }
    oadrive::vision::PatchesToLookFor& patchesToLookFor()          { return m_patches_to_look_for_worker; }
    double& matchingThreshold()                           { return m_matching_threshold_worker; }

    //! used to be able to only spawn new thread when old one is ready again
    void setReady(bool isReady) { m_ready = isReady; }
    bool isReady() const { return m_ready; }

    //! determining which treshold should be used
    Threshold m_threshold_to_use;


  private:
    //! overwrite ThreadFunc of cKernelThread, this will be called in a loop
    tResult ThreadFunc();

    //! do image processing on data set
    void process();

    //! bool to know if thread is ready to be used for next spawned thread
    bool m_ready;

    //! count the frames to recalibrate the threshold.
    int m_framecount;

    //! build consistent track in pixel coordinates, using information from consistent_patches input pin
    vector<TrackSegment> buildConsistentPatches();

    //! used to find track ahead of car based on a belief and the current camera data
    Stitcher m_stitcher;

    //!
    ImagePreprocessor* m_imagePreprocessor;

    //! Copy of vehicle pose for worker thread
    oadrive::core::Pose2d m_vehicle_pose_worker;

    //! state flags, that decides how the stitcher should behave
    katana::PerceptionState m_perception_state_worker;
    u_int8_t m_number_of_stitches_worker;
    oadrive::vision::PatchesToLookFor m_patches_to_look_for_worker;
    double m_matching_threshold_worker;

    //! Copy of consistent patches for worker thread
    std::vector<katana::sPatch> m_consistent_patches_worker;

    //! Current image
    cv::Mat m_image_worker;

    //! Function to transmit patches
    PatchesTransmitFunc m_transmit_func;

    //! whether to use binary or HSV threshold
//    bool m_use_binary_threshold;

//    //! whether to use HSV threshold
//    bool m_use_hsv_threshold;

//    //! whether to use histogram threshold
//    bool m_use_histogram_treshold;

  };

  //! Data available, processing requested -> set data to cImageProcessing and start processing
  tResult prepareProcessing();

#ifdef KATANA_LT_MEASURE_PERFORMANCE
  size_t m_performance_counter = 0;
  std::chrono::system_clock::time_point m_last_timestamp;
#endif

  //! Transmit given patches
  tResult transmitPatches(const vector<katana::sPatch>& patches, katana::PerceptionState perception_state);


  //! thread/class used to process input image
  std::shared_ptr<cImageProcessing> m_oProcessingThread;

  //! used to skip first few frames to let camera adjust to light etc
  static constexpr u_int32_t FRAMES_TO_BE_SKIPPED = 90;
  u_int32_t m_framesAlreadySkipped;

  //! binary threshold used for IPM. pixel with higher intensity will be white, others black
  u_int8_t m_binary_threshold;

  //! current image from input pin
  cv::Mat m_image;

  //! last get Pose
  oadrive::core::Pose2d m_vehicle_pose;
      
  //! Mutex for accessing pose
  std::mutex m_pose_mutex;

  //! points to the latest array of patches that may be in current view of car, obtained by Mission Control
  katana::sPatch* m_consistent_patches;     //!< struct to receive from pin
  u_int32_t m_consistent_patches_size;      //!< size of array

  //! state/command flags from mission control
  katana::PerceptionState m_perception_state;
  u_int8_t m_number_of_stitches;
  oadrive::vision::PatchesToLookFor m_patches_to_look_for;
  double m_matching_threshold;

  //! variable to remember if new image arrived during execution/idling of thread
  bool m_request_processing_of_next_image;

  //! input format
  tBitmapFormat m_sInputFormat;


  /** ******* PINS AND MEDIA DESCRIPTION ***********/
  //! Input pin for UDP listening (used to give commands to this filter for debugging)
  cInputPin m_iDebug;


  //! Input video from xtion
  cVideoPin m_oInputPin;

  //! output pin for found patches
  cOutputPin m_opin_patches;

  //! pose that contains the latest odometry values
  cInputPin m_ipin_pose;

  //! array of patches that may be in current view of car, to have a starting point for stitching. received by Mission Control
  cInputPin m_ipin_patches;

  //! Coder Descriptor for the output pins
  cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputPatch;

};

//*************************************************************************************************
#endif // _LANETRACKER_HEADER_

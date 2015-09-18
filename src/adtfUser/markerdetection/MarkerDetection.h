#ifndef _MARKER_DETECTION_FILTER_HEADER_
#define _MARKER_DETECTION_FILTER_HEADER_


#define OID_ADTF_MARKER_DETECTION "adtf.aadc.katana.makerDetection"

//#define KATANA_MD_WRITE_MARKERS

#include "katanaCommon/katanaCommon.h"
#include "PinConversions.h"


#include <oadrive_markers/RoadSignDetector.h>

#include <queue>

class cMarkerDetection : public adtf::cAsyncDataTriggeredFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_MARKER_DETECTION, "KATANA Marker Detection Filter", OBJCAT_DataFilter, "Marker Detection Filter Version", 2, 2, 20, "MARGE")


//    typedef std::array<SignQueue, (u_int32_t)katana::TrafficSign::SIGN_COUNT> SignArray;

    public:
      cMarkerDetection(const tChar* __info);
      virtual ~cMarkerDetection();

      tResult OnAsyncPinEvent(IPin* pSource,
		  tInt nEventCode,
		  tInt nParam1,
		  tInt nParam2,
		  IMediaSample* pMediaSample);
      tResult Init(tInitStage eStage, __exception = NULL);

    private:

      class MarkerAverage
      {
      public:
        MarkerAverage() {}
        ~MarkerAverage() {}

        static constexpr u_int8_t CHECK_FRAMES = 5;
        static constexpr u_int8_t NEED_FRAMES = 3;

        void addFrame(const oadrive::markers::RoadSignsContainer& markers_in_frame);
        bool checkFramesForMostSeen(int& marker_id);

      private:

        typedef std::queue<bool> IDFrames;
        struct IDCounter
        {
          IDFrames frames;
          u_int8_t counter;
          bool seen;

          IDCounter()
          {
            counter = 0;
            seen = true;
          }

          void frame()
          {
            frames.push(seen);
            counter += (u_int8_t)seen;
            if (frames.size() > CHECK_FRAMES)
            {
              counter -= (u_int8_t)frames.front();
              frames.pop();
            }
            seen = false;
          }

        };
        typedef std::map<int, IDCounter> IDCounterMap;


        IDCounterMap m_id_counter;

        void addNewIDCounter(int id)      { m_id_counter[id] = IDCounter(); }
      };

      //! Transmit the road signs over the pin
      tResult sendRoadSignStruct(const vector<katana::sRoadSign>& roadSigns);

      tResult loadDictionary();

      tResult ProcessInput(IMediaSample* pSample);
      void sendMarkers(const aruco::Marker& marker, bool appearedMarker = true);
      void averageMarkers(oadrive::markers::RoadSignsContainer detectedRoadSigns);
      oadrive::markers::RoadSignsContainer filterInvalidSigns(oadrive::markers::RoadSignsContainer& roadSigns);
      katana::TrafficSign getSignFromMarker(aruco::Marker m);

      /*! input Pin for video */
      cVideoPin m_oPinInputVideo;

      /*! output Pin for detected Sign */
      cOutputPin m_oPinRoadSign;

      //! pose that contains the latest odometry values
      cInputPin m_ipin_pose;

      /*! bitmapformat of input image */
      tBitmapFormat      m_sInputFormat;

      //! Counter for frames
      int32_t m_counter;

      //! Is last marker set
      bool m_last_marker_set;

      //! Last best marker
      aruco::Marker m_last_marker;

      //! Detected signs in last frames
      //SignArray m_signs;

      //! last get Pose
      oadrive::core::Pose2d m_vehicle_pose;

      //! The marker detedtor
      oadrive::markers::RoadSignDetector m_roadSignDetector;

      //! Dictionary for aruco
      aruco::Dictionary m_dictionary;

      //! MarkerAverage
      MarkerAverage m_marker_average;


      float m_min_parking_sign_area;
      float m_max_parking_sign_area;

      float m_min_sign_area;
      float m_max_sign_area;

      float m_min_sign_aspect_ratio;
      float m_max_sign_aspect_ratio;
};

#endif // _MARKER_DETECTION_FILTER_HEADER_

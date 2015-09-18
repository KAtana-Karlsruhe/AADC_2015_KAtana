/**
 *
 * ADTF Demo Source.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: WNEROLF $
 * $Date: 2012-05-30 10:34:13 +0200 (Wed, 30 May 2012) $
 * $Revision: 31776 $
 *
 * @remarks
 *
 */

#ifndef _OBSTACTLE_DETECTION_FILTER_CLASS_HEADER_
#define _OBSTACTLE_DETECTION_FILTER_CLASS_HEADER_

#include "IPM.h"
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>

#include "Pose.h"

using namespace std;
using namespace cv;
#define OID_ADTF_OBSTACTLE_DETECTION "adtf.aadc.obstactle.detection"

class cObstactleDetectionFilter : public cFilter
{
    ADTF_FILTER(OID_ADTF_OBSTACTLE_DETECTION, "KATANA Obstactle Detection", OBJCAT_Converter)

//    struct Trackpoint {
//        katana::Pose pose;
//        katana::PatchType type;
//    };

//    typedef vector<Trackpoint> Track;

    protected:

        //! Pins
        cVideoPin     m_oInputPin;
        cVideoPin     m_oOutputPin;

        tBitmapFormat m_sInputFormat;
        tBitmapFormat m_sOutputFormat;


        cOutputPin    m_opin_patch;

        cInputPin     m_ipin_pose;
        //! Coder Descriptor for the output pins
//        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputPatch;

        //! Coder Descriptor for the input pins
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalPoseInput;
        katana::Pose m_vehicle_pose;

    public:
        cObstactleDetectionFilter(const tChar* __info);
        virtual ~cObstactleDetectionFilter();
        tResult CreateOutputPins(__exception = NULL);
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
//        tResult Stop(__exception = NULL);
//        tResult Shutdown(tInitStage eStage, __exception = NULL);

    public:
        tResult OnPinEvent(IPin* pSource,
                           tInt nEventCode,
                           tInt nParam1,
                           tInt nParam2,
                           IMediaSample* pIMediaSamples);

    protected:
        tResult Process(IMediaSample* pISample);

//	private:
//        class cProcessingThread : public cKernelThread
//        {
//            protected:
//                cObstactleDetectionFilter*    m_pParent;

//            public:
//                cProcessingThread();
//                tResult SetParent(cObstactleDetectionFilter* pParent);

//            protected: //overwrite ThreadFunc of cKernelThread
//                tResult ThreadFunc();
//        };

//        cProcessingThread              m_oProcessingThread;
//        IMediaSample*                  currentPISample;

        //! source and destination coordinates for IPM
		tInt sUpperLeftX;
		tInt sUpperLeftY;

		tInt sUpperRightX;
		tInt sUpperRightY;

		tInt sLowerLeftX;
		tInt sLowerLeftY;

		tInt sLowerRightX;
		tInt sLowerRightY;

		tInt dUpperLeftX;
		tInt dUpperLeftY;

		tInt dUpperRightX;
		tInt dUpperRightY;

		tInt dLowerLeftX;
		tInt dLowerLeftY;

		tInt dLowerRightX;
		tInt dLowerRightY;

        //! forward and backward stitching
        static const int FIRST_ENVIRONMENT_SIZE = 7;
        static const int NORMAL_ENVIRONMENT_SIZE = 3;

        //!
        int m_lastRotation;

        //! ratio that is choosen for IPM
        tInt m_pixelToCentimeterRatio;

        //! x-translation that is done after IPMing
        tInt m_ipmTranslation;
				
        //! treshold value for binary thresholding
        tInt m_thresholdValue;
		
        //! Inverse Perspective Mapping
//		IPM m_IPM;

        //! patches that we are interested in to find. a combination of different types and rotations
//        vector<Patch> m_patches;

        //! patch mapping from patchType to a vector of patches from the given type.
//        std::map<katana::PatchType,vector<Patch>> m_patchMapping;

        //! initialize the IPM with source and destination coordinates
//		void initializeIPM();

        //! initialize m_patches with desired Patch types and rotations
//        void initializePatches();

        //! do preprocessing prior to IPM
		void preprocessImage(Mat input, Mat &output);

        //! threshold the input matrix
		void doBinaryThreshold(Mat input, Mat &output, int threshold_value);

        //! erode and dilate white pixels
		void erodeAndDilate(Mat input, Mat &output, int dilation_type, int dilation_size);

        //! guess track for controller block by stitching patches
//        Track guessTrack(Mat &input);

        //! MediaSample for the thread
//        IMediaSample* getMediaSample();

        //! add a border to input matrix to allow for the detection of unusual road positions
//        void addBorder(Mat &input, int top, int left, int bottom, int right, Mat &output);

        //! guess first stitch Location
//        Trackpoint findMatchInSearchspace(Mat &input, Mat &searchSpace, Point searchSpaceOffset, Point borderOffset, bool firstStitch, Patch &matchedPatch, Point &stitch, bool backward = false);

        //! choose proper ROI dependent on stitch location
//        Mat prepareSearchspace(Mat &input, Point stitchLocation, Point &upperLeftOfROI);

        //! pattern matching
//        Point matchPattern(Mat &input, Mat pattern);
//        Point matchPatterns(Mat &searchSpace, vector<Patch> patches, Patch &matchedPatch);
//        void drawRectangle(Mat& input, Point topLeft, int width, int height, int color);
//        vector<Patch> calculatePatchEnvironment(katana::PatchType type, int rotation, int numberOfElements);
//        vector<Patch> calculatePatchesOfInterest(bool firstStitch = false);

        //! transmit media sample to controller block
//        tResult TransmitMediaSample(Trackpoint trackpoint);

        //! coordinate conversions
        Point toCarCoordinates(Point picCoordinates);
        Point toPictureCoordinates(Point subPicCoordinate, Point origin);
        Point borderedInputToWorld(Point borderedInputCoordinates, Point borderOffset);
};

//*************************************************************************************************
#endif // _LANETRACKER_FILTER_CLASS_HEADER_

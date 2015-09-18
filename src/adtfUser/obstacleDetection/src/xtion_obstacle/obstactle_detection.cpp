/**
 *
 * ADTF Demo Source.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: A1FORDM $
 * $Date: 2013-02-06 17:08:10 +0100 (Wed, 06 Feb 2013) $
 * $Revision: 36869 $
 *
 * @remarks
 *
 */

#include "stdafx.h"
#include "obstactle_detection.h"
#include <vector>
#include <iostream>
#include <cmath>

ADTF_FILTER_PLUGIN("KATANA Obstactle Detection", OID_ADTF_OBSTACTLE_DETECTION, cObstactleDetectionFilter)

//cObstactleDetectionFilter::cProcessingThread::cProcessingThread():
//        cKernelThread(),
//        m_pParent(NULL)
//{
//}

//tResult cObstactleDetectionFilter::cProcessingThread::SetParent(cObstactleDetectionFilter* pParent)
//{
//    m_pParent = pParent;
//    RETURN_NOERROR;
//}

/**
 * The thread function which will be called in a loop.
 * @return Standard Result Code.
 */
//tResult cObstactleDetectionFilter::cProcessingThread::ThreadFunc()
//{
//    IMediaSample* cur = m_pParent->getMediaSample();
//    tResult ret = m_pParent->Process(cur);
//    this->Suspend();
//    return ret;
//}

cObstactleDetectionFilter::cObstactleDetectionFilter(const tChar* __info) : cFilter(__info)
{
    cMemoryBlock::MemSet(&m_sOutputFormat, 0, sizeof(m_sOutputFormat));
    cMemoryBlock::MemSet(&m_sInputFormat, 0, sizeof(m_sInputFormat));
	
    m_pixelToCentimeterRatio = 3;
    m_ipmTranslation = 213;

    SetPropertyInt("pixelToCentimeterRatio", m_pixelToCentimeterRatio);
    SetPropertyInt("ipmTranslation", m_ipmTranslation);

    SetPropertyInt("sUpperLeftX", 216);
    SetPropertyInt("sUpperLeftY", 219);
    SetPropertyInt("sUpperRightX", 457);
    SetPropertyInt("sUpperRightY", 219);
    SetPropertyInt("sLowerLeftX", 92);
    SetPropertyInt("sLowerLeftY", 380);
    SetPropertyInt("sLowerRightX", 613);
    SetPropertyInt("sLowerRightY", 380);

    SetPropertyInt("dUpperLeftX", 20*m_pixelToCentimeterRatio+m_ipmTranslation);
    SetPropertyInt("dUpperLeftY", 90*m_pixelToCentimeterRatio);
    SetPropertyInt("dUpperRightX", 60*m_pixelToCentimeterRatio+m_ipmTranslation);
    SetPropertyInt("dUpperRightY", 90*m_pixelToCentimeterRatio);
    SetPropertyInt("dLowerLeftX", 20*m_pixelToCentimeterRatio+m_ipmTranslation);
    SetPropertyInt("dLowerLeftY", 140*m_pixelToCentimeterRatio);
    SetPropertyInt("dLowerRightX", 60*m_pixelToCentimeterRatio+m_ipmTranslation);
    SetPropertyInt("dLowerRightY", 140*m_pixelToCentimeterRatio);

    SetPropertyInt("threshold", 120);
}

cObstactleDetectionFilter::~cObstactleDetectionFilter()
{
}

tResult cObstactleDetectionFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

m_pixelToCentimeterRatio = GetPropertyInt("pixelToCentimeterRatio");
		m_ipmTranslation = GetPropertyInt("ipmTranslation");

		// source points for homography
		sUpperLeftX = GetPropertyInt("sUpperLeftX");
		sUpperLeftY = GetPropertyInt("sUpperLeftY");

		sUpperRightX = GetPropertyInt("sUpperRightX");
		sUpperRightY = GetPropertyInt("sUpperRightY");

		sLowerLeftX = GetPropertyInt("sLowerLeftX");
		sLowerLeftY = GetPropertyInt("sLowerLeftY");

		sLowerRightX = GetPropertyInt("sLowerRightX");
		sLowerRightY = GetPropertyInt("sLowerRightY");


		// destination points for homography
		dUpperLeftX = GetPropertyInt("dUpperLeftX");
		dUpperLeftY = GetPropertyInt("dUpperLeftY");

		dUpperRightX = GetPropertyInt("dUpperRightX");
		dUpperRightY = GetPropertyInt("dUpperRightY");

		dLowerLeftX = GetPropertyInt("dLowerLeftX");
		dLowerLeftY = GetPropertyInt("dLowerLeftY");

		dLowerRightX = GetPropertyInt("dLowerRightX");
		dLowerRightY = GetPropertyInt("dLowerRightY");
				
		// threshold maxvalue for binary thresholding
		m_thresholdValue = GetPropertyInt("threshold");

		// build transformation needed for Inverse Perspective Mapping
//		initializeIPM();

    if (eStage == StageFirst)
    {	
//		// build needed patches
//		initializePatches();

        // relevant output pins
//        CreateOutputPins();

        //! Input pins
        // as input we get RGB images
        tResult nResult = m_oInputPin.Create("input",
            IPin::PD_Input, static_cast<IPinEventSink*>(this));
        RETURN_IF_FAILED(nResult);

        RETURN_IF_FAILED(RegisterPin(&m_oInputPin));

        // debug output video pin
        nResult = m_oOutputPin.Create("output", IPin::PD_Output, static_cast<IPinEventSink*>(this));

        RETURN_IF_FAILED(nResult);
	
        RETURN_IF_FAILED(RegisterPin(&m_oOutputPin));

        // pose input
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tPose");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> m_pTypeSignalPose = new cMediaType(0, 0, 0, "tPose", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_pTypeSignalPose->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalPoseInput));

        RETURN_IF_FAILED(m_ipin_pose.Create("pose_input", m_pTypeSignalPose, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_ipin_pose));

//	m_oProcessingThread.SetParent(this);
//        m_oProcessingThread.Create(cProcessingThread::TF_Suspended);
    }
    
    else if (eStage == StageGraphReady)
    {
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

        const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
        if (pFormat == NULL)
        {
            LOG_ERROR("No Bitmap information found on pin \"input\"");
            RETURN_ERROR(ERR_NOT_SUPPORTED);
        }
        cMemoryBlock::MemCopy(&m_sInputFormat, pFormat, sizeof(tBitmapFormat));
        cMemoryBlock::MemCopy(&m_sOutputFormat, pFormat, sizeof(tBitmapFormat));

        m_oOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }

    RETURN_NOERROR;
}

tResult cObstactleDetectionFilter::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}


//tResult cObstactleDetectionFilter::Stop(__exception)
//{
//    m_oProcessingThread.Suspend();
//    return cFilter::Stop(__exception_ptr);
//}

//tResult cObstactleDetectionFilter::Shutdown(tInitStage eStage, __exception)
//{
//    if (eStage == StageNormal)
//    {
//        m_oProcessingThread.Release();
//    }
//    return cFilter::Shutdown(eStage, __exception_ptr);
//}

tResult cObstactleDetectionFilter::OnPinEvent(IPin* pSource,
                                         tInt nEventCode,
                                         tInt nParam1,
                                         tInt nParam2,
                                         IMediaSample* pIMediaSample)
{
    if (pSource == &m_oInputPin && nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
//        if(m_oProcessingThread.GetState() == cProcessingThread::TS_Suspended) {
//        currentPISample = pIMediaSample;
//        m_oProcessingThread.Run();
//        }
			Process(pIMediaSample);
    } 
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
            cMemoryBlock::MemCopy(&m_sOutputFormat, pFormat, sizeof(tBitmapFormat));
            m_oOutputPin.SetFormat(&m_sOutputFormat, NULL);
        }
    }
    else if (pSource == &m_ipin_pose && nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
      cObjectPtr<IMediaCoder> pCoderPose;
      RETURN_IF_FAILED(m_pCoderDescSignalPoseInput->Lock(pIMediaSample, &pCoderPose));

      katana::_position_type yPoseInput = 0;
      pCoderPose->Get("tPoint.yCoordinate",(tVoid*)&yPoseInput);
      m_vehicle_pose.setY(yPoseInput);
      katana::_position_type xPoseInput = 0;
      pCoderPose->Get("tPoint.xCoordinate",(tVoid*)&xPoseInput);
      m_vehicle_pose.setX(xPoseInput);
      katana::_angle_type psiPoseInput = 0;
      pCoderPose->Get("theta",(tVoid*)&psiPoseInput);
      m_vehicle_pose.setTheta(psiPoseInput);

      m_pCoderDescSignalPoseInput->Unlock(pCoderPose);
    }

    RETURN_NOERROR;
}

tResult cObstactleDetectionFilter::Process(IMediaSample* pISample)
{
    RETURN_IF_POINTER_NULL(pISample);

    cObjectPtr<IMediaSample> pNewSample;
    RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));

    RETURN_IF_FAILED(pNewSample->AllocBuffer(m_sOutputFormat.nSize));

    const tVoid* l_pSrcBuffer;
    tVoid* l_pDestBuffer;
    

    if (IS_OK(pISample->Lock(&l_pSrcBuffer)))
    {
        if (IS_OK(pNewSample->WriteLock(&l_pDestBuffer)))
        {
			Mat src;	
			src.create(m_sInputFormat.nHeight, m_sInputFormat.nWidth, CV_8UC1);	
			src.data = (uchar*)l_pSrcBuffer;

//			// initialisation
//			IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
//			img->imageData = (char*)l_pSrcBuffer;
//			Mat src(cvarrToMat(img));
//			cvReleaseImage(&img);
			std::cout << "src " << src.at<u_int16_t>(50,50) << std::endl;

			//  the image, convert color space, threshold, erode and dilate
			preprocessImage(src, src);

			// put our src matrix on the output pin

			// put our src matrix on the output pin
//			Mat mat = cv::Mat(m_sOutputFormat.nHeight, m_sOutputFormat.nWidth, CV_8UC3,l_pDestBuffer);
//			image.convertTo(image, CV_16UC1);
			Mat mat = cv::Mat(m_sOutputFormat.nHeight, m_sOutputFormat.nWidth, CV_16UC1, l_pDestBuffer);
//			Mat mat = cv::Mat(m_sOutputFormat.nHeight, m_sOutputFormat.nWidth, CV_16UC1, l_pDestBuffer);
//            cvtColor(src, src, CV_GRAY2BGR);
            src.copyTo(mat);



pNewSample->SetTime(pISample->GetTime());

    RETURN_IF_FAILED(m_oOutputPin.Transmit(pNewSample));
			pNewSample->Unlock(l_pDestBuffer);
        }

        pISample->Unlock(l_pSrcBuffer);
    }
    RETURN_NOERROR;
}

void cObstactleDetectionFilter::preprocessImage(Mat input, Mat &output) {	
	// convert color space
//	cvtColor(input, input,CV_BGR2GRAY);
	
	// threshold
//    doBinaryThreshold(temp, temp, m_thresholdValue);
	
	// erode and dilate
	erodeAndDilate(input, input, MORPH_CROSS, 1);	
//	cvCvtColor(output,output,CV_RGB2GRAY);

    Mat image;
    image = imread("/media/odroid/6538-3462/Obstactle_filter_recording/rec2_bmp/output_0000000007703351.bmp", 0);
	image.convertTo(image, CV_16UC1);


//	const Mat A( 10, 20, CV_32FC1, Scalar::all(CV_PI) );
//	const Mat B( A.size(), CV_8UC1, Scalar::all(10) );
//	Mat C(A.size(), A.type());
	for (int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
//			std::cout << "size " << i << " " << j << std::endl;
		    output.at<u_int16_t>(i,j) = input.at<u_int16_t>(i,j) - image.at<u_int16_t>(i,j);
		//	std::cout << "input " << input.at<u_int16_t>(i,j) << " image " << image.at<u_int16_t>(i,j) << " output " << output.at<u_int16_t>(i,j) << std::endl;

			output.at<u_int16_t>(i,j) = ((output.at<u_int16_t>(i,j) > 255) ? 0 : output.at<u_int16_t>(i,j));
	//		output.at<uchar>(i,j) = output.at<u_int16_t>(i,j) >= 30?255:0;
		}
	}


	//std::cout << "image " << image.cols << " " << image.rows << std::endl;
	//std::cout << "output " << output.cols << " " << output.rows << std::endl;
//	Mat asd = output;	
//  output = asd - image;
//	output.convertTo(output, CV_8UC1);

//	threshold(output, output, 15, 255,THRESH_BINARY);
//	output.convertTo(output, CV_16UC1);
//	cvCvtColor(output,output,CV_GRAY2BGR);

}

void cObstactleDetectionFilter::doBinaryThreshold(Mat input, Mat &output, int threshold_value) {
	threshold(input, output, threshold_value, 255,THRESH_BINARY);
}

void cObstactleDetectionFilter::erodeAndDilate(Mat input, Mat &output, int dilation_type, int dilation_size) {
	Mat kernel = getStructuringElement(dilation_type, Size(2*dilation_size + 1, 2*dilation_size + 1), Point(dilation_size, dilation_size));
	
	Mat temp = input;
//	erode(input, temp, kernel, Point(-1,-1), 3);
	dilate(temp, output, kernel, Point(-1,-1), 3);
}

//void cObstactleDetectionFilter::initializeIPM() {
//	int height = m_sInputFormat.nHeight;
//	int width = m_sInputFormat.nWidth;	

//	// The four source points
//	vector<Point2f> origPoints;			
//	origPoints.push_back( Point2f(sUpperLeftX, sUpperLeftY) );
//	origPoints.push_back( Point2f(sUpperRightX, sUpperRightY ));
//	origPoints.push_back( Point2f(sLowerLeftX, sLowerLeftY) );
//	origPoints.push_back( Point2f(sLowerRightX, sLowerRightY) );

//	// The four corresponding points in the destination image
//	vector<Point2f> destPoints;
//	destPoints.push_back( Point2f(dUpperLeftX, dUpperLeftY) );
//	destPoints.push_back( Point2f(dUpperRightX, dUpperRightY ));
//	destPoints.push_back( Point2f(dLowerLeftX, dLowerLeftY) );
//	destPoints.push_back( Point2f(dLowerRightX, dLowerRightY) );

//	// initialize m_IPM
//	IPM ipm( Size(width, height), Size(width, height), origPoints, destPoints );
//	m_IPM = ipm;
//}

//Point cObstactleDetectionFilter::toPictureCoordinates(Point subPicCoordinate, Point origin) {
//    return subPicCoordinate + origin;
//}

//Point cObstactleDetectionFilter::toCarCoordinates(Point picCoordinates) {
//    Point middleOfCar(333,452);

//    Point blindSpotLength(0,6*m_pixelToCentimeterRatio);
//    Point carLength(0,45*m_pixelToCentimeterRatio);
//    Point carCoordinates = (middleOfCar + blindSpotLength + carLength) - picCoordinates;

//    return carCoordinates;
//}

//tResult cObstactleDetectionFilter::TransmitMediaSample(Trackpoint trackpoint) {
//    cObjectPtr<IMediaSample> media_sample;
//    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&media_sample));

//    cObjectPtr<IMediaSerializer> serializer;
//    RETURN_IF_FAILED(m_pCoderDescSignalOutputPatch->GetMediaSampleSerializer(&serializer));
//    tInt n_size = serializer->GetDeserializedSize();
//    RETURN_IF_FAILED(media_sample->AllocBuffer(n_size));

//    cObjectPtr<IMediaCoder> patch;

//    RETURN_IF_FAILED(m_pCoderDescSignalOutputPatch->WriteLock(media_sample, &patch));
//    
//    katana::_position_type tmp = trackpoint.pose.getX();
//    patch->Set("tPose.tPoint.xCoordinate", (tVoid*)&tmp);
//    
//    tmp = trackpoint.pose.getY();
//    patch->Set("tPose.tPoint.yCoordinate", (tVoid*)&tmp);
//    
//    katana::_angle_type theta = trackpoint.pose.getTheta();
//    patch->Set("tPose.theta", (tVoid*)&theta);

//    katana::PatchType type = trackpoint.type;
//    patch->Set("type", (tVoid*)&type);
//    
//    m_pCoderDescSignalOutputPatch->Unlock(patch);

//    //transmit
//    media_sample->SetTime(_clock->GetStreamTime());
//    m_opin_patch.Transmit(media_sample);

//    RETURN_NOERROR;
//}

//tResult cObstactleDetectionFilter::CreateOutputPins(__exception)
//{
//  cObjectPtr<IMediaDescriptionManager> pDescManager;
//  RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
//    
//  tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tPatch");
//  RETURN_IF_POINTER_NULL(strDescSignalValue);
//  cObjectPtr<IMediaType> m_pTypeSignalValue = new cMediaType(0, 0, 0, "tPatch", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
//  RETURN_IF_FAILED(m_pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputPatch));
//	
//  RETURN_IF_FAILED(m_opin_patch.Create("patch", m_pTypeSignalValue, static_cast<IPinEventSink*> (this)));

//  RETURN_IF_FAILED(RegisterPin(&m_opin_patch));

//  RETURN_NOERROR;
//}

//void cObstactleDetectionFilter::drawRectangle(Mat& input, Point topLeft, int width, int height, int color) {
//    rectangle(input, topLeft, Point(topLeft.x + width, topLeft.y + height), Scalar(color,color, color), 2);
//}


//IMediaSample* cObstactleDetectionFilter::getMediaSample() {
//    return this->currentPISample;
//}

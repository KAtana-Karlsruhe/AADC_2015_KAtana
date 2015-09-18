/**********************************************************************
* Copyright (c) 2013 BFFT Gesellschaft fuer Fahrzeugtechnik mbH.
* All rights reserved.
**********************************************************************
* $Author:: spiesra $  $Date:: 2014-09-16 13:29:48#$ $Rev:: 26104   $
**********************************************************************/

/*! \brief Xtion Camera
 *         
 *  Dieser Filter liefert die Videobilder einer angeschlossen Asus Xtion Kamera. Als Ausgang liefert er das über die Eigenschaften eingestellte Videoformat am Ausgang Video_RGB, ein Videobild mit niedriger Auflösung (320x180 30 FPS) am Pin Video_RGB_LowRes und das Tiefenbild am Pin Depth_Image mit gleichem Format wie in den Einstellungen definiert.
 */

#ifndef _XTION_CAPTURE_FILTER_HEADER_
#define _XTION_CAPTURE_FILTER_HEADER_



#define OID_ADTF_XTION_CAPTURE "adtf.aadc.xtionCaptureDevice"
//! class of capturing the image dat
    /*!
    this filter reads the stream from the xtion device
    */
class cXtionCaptureFilter : public adtf::cFilter, public adtf::IKernelThreadFunc
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_XTION_CAPTURE, "AADC Xtion Camera", OBJCAT_CameraDevice, "AADC Xtion Camera", 1, 1, 0, "");    
    public:
        cXtionCaptureFilter(const tChar*);
        virtual ~cXtionCaptureFilter();
        tResult Init(tInitStage eStage, __exception=NULL);
        tResult Start(ucom::IException** __exception_ptr=NULL);
        tResult Stop(ucom::IException** __exception_ptr=NULL);
        tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
    protected:
        /*! the threadfunc which handles the reading and sending of the data
        @param pThread
        @param pvUserData
        @param szUserData
        */
        tResult ThreadFunc(cKernelThread* pThread, tVoid* pvUserData, tSize szUserData);
        
        /*! 
        the function for reading the data
        */
        tResult ReadData();
        
        /*!
        function to load an xml to set the settings of the xtion
        */
        tResult LoadConfiguration();
        
        tInt        m_nWidthVideo;                /**< width of the stream */
        tInt        m_nHeightVideo;                /**< height of the stream */
        tInt        m_nWidthDepth;                /**< width of the depth stream */
        tInt        m_nHeightDepth;                /**< height of the depth stream */
        tInt        m_FPSVideo;                    /**< frames per second of the video stream */
        tInt        m_FPSDepth;                    /**< frames per second of the depth stream */
        
        cVideoPin    m_oRGBVideo;                /**<  outputpin for the RGB video */
        cVideoPin    m_oDepthImage;                /**<  outputpin for the depth images */
        
        tBitmapFormat    m_sBitmapFormat;        /**< bitmap format for the RGB video */
        tBitmapFormat    m_sBitmapFormatDepth;    /**< bitmap format for the depth video */
        
        tBool           m_setRegistration;        /**< enable flag for setRegistration */
        tBool           m_setDepthColorSync;    /**< enable flag for setDepthColorSync */
        tBool             m_setDepthInMillimeter;     /**< if true the values in the depth image are given in millimeter*/
        
        cKernelThread    m_oWorker;                /**< the worker for the thread */
        
        xtionGrabber    m_xtionGrabber;            /**< the instance of the grabber (only linux) */
        #ifdef _WIN32
        cv::Mat        m_emptyMat;                    /**< the empty matrix which is transmitted in windows */
        #endif
        tInt        m_GrabberTimeoutMs;            /**< the time out for the grabber */        
        
        cFilename m_fileConfig;                    /**< holds the xml file for the supporting points */
};

#endif // _XTION_CAPTURE_FILTER_HEADER_

#include "stdafx.h"
#include "xtionGrabber.h"
#include "xtioncapturefilter.h"
/**
 *
 * ADTF Xtion Capture
 *
 *
 * $Author: spiesra
 * $Date: 2013-06-17
 *
 *
 */  

#ifdef GetObject
    // so we undef it.
    #undef GetObject
#endif

ADTF_FILTER_PLUGIN("Xtion Capture Device", OID_ADTF_XTION_CAPTURE, cXtionCaptureFilter)

cXtionCaptureFilter::cXtionCaptureFilter(const tChar* __info) : cFilter(__info) 
{
    m_GrabberTimeoutMs=1000;
    
    SetPropertyStr("Configuration File",""); 
    SetPropertyBool("Configuration File" NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr("Configuration File" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");     
}

cXtionCaptureFilter::~cXtionCaptureFilter()
{
}

tResult cXtionCaptureFilter::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
    {
        //create the output pins of the filter
        RETURN_IF_FAILED(m_oRGBVideo.Create("Video_RGB", adtf::IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oRGBVideo));

        RETURN_IF_FAILED(m_oDepthImage.Create("Depth_Image", adtf::IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oDepthImage));
    }
    else if (eStage == StageNormal)
    {
        //load the configuration file for the settings of xtion camera    
        tResult nResult;
        nResult = LoadConfiguration(); 
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult,"Failed to load the configuration file for the xtion");          
        
        //set the videoformat of the rgb video pin
        m_sBitmapFormat.nWidth = m_nWidthVideo;
        m_sBitmapFormat.nHeight = m_nHeightVideo;
        m_sBitmapFormat.nBitsPerPixel = 24;
        m_sBitmapFormat.nPixelFormat = cImage::PF_RGB_888;
        m_sBitmapFormat.nBytesPerLine = m_nWidthVideo * 3;
        m_sBitmapFormat.nSize = m_sBitmapFormat.nBytesPerLine * m_nHeightVideo;
        m_sBitmapFormat.nPaletteSize = 0;
        m_oRGBVideo.SetFormat(&m_sBitmapFormat, NULL);
        
        //set the videoformat of the depth video pin
        m_sBitmapFormatDepth.nWidth = m_nWidthDepth;
        m_sBitmapFormatDepth.nHeight = m_nHeightDepth;
        m_sBitmapFormatDepth.nBitsPerPixel = 16;
        m_sBitmapFormatDepth.nPixelFormat = cImage::PF_GREYSCALE_16;
        m_sBitmapFormatDepth.nBytesPerLine = m_nWidthDepth * 2;
        m_sBitmapFormatDepth.nSize = m_sBitmapFormatDepth.nBytesPerLine * m_nHeightDepth;
        m_sBitmapFormatDepth.nPaletteSize = 0;
        m_oDepthImage.SetFormat(&m_sBitmapFormatDepth, NULL);    
    }
    else if (eStage == StageGraphReady)    
    {        
        //check which streams has to be enabled in th xtion
        tBool videoEnabled = m_oRGBVideo.IsConnected();        
        tBool depthEnabled = m_oDepthImage.IsConnected();

        // if no output pin is connected the depth image is enabled because one stream must be set to tTrue;
        if (videoEnabled==tFalse && depthEnabled==tFalse) depthEnabled = tTrue;
            
        // initialize the grabber with the videoformats, which streams has the be enabled, the framerates and some more options
        if (!(m_xtionGrabber.initialize(m_nWidthDepth,m_nHeightDepth,m_FPSDepth,m_nWidthVideo,m_nHeightVideo,m_FPSVideo,depthEnabled,videoEnabled,m_setDepthInMillimeter,m_setRegistration,m_setDepthColorSync)))
            THROW_ERROR_DESC(ERR_DEVICE_NOT_READY,"Failed to initialize Xtion!")
        else
            LOG_INFO(cString::Format("Xtion Capture Device: Grabber is now initialized. Depth mode: %d Video mode: %d",depthEnabled,videoEnabled));
        // creates the thread for the reading of data from the xtion
        tResult nResult;
        nResult = m_oWorker.Create(cKernelThread::TF_Suspended, static_cast<adtf::IKernelThreadFunc*>(this));
        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult,"Failed to create thread for xtion");
        }

    RETURN_NOERROR;
}

tResult cXtionCaptureFilter::Start(ucom::IException** __exception_ptr)
{        
    // check if worker is running
    if (m_oWorker.GetState() != cKernelThread::TS_Running)
        {
        // check if grabber is initialized and can be started
        if(m_xtionGrabber.isInitialized() && m_xtionGrabber.start())
        {
            //check if grabber is running
            if (m_xtionGrabber.isRunning()) 
                LOG_INFO(cString::Format("Xtion Capture Device: Grabber is now running."));
            else
                {
                    LOG_ERROR("Xtion Capture Device: Unable to run OpenNI2Grabber, program terminating!");
                    THROW_ERROR_DESC( ERR_DEVICE_NOT_READY, "Unable to run Xtion Grabber")
                }                        
        }
        else
        {
            LOG_ERROR("Xtion Capture Device: Unable to start OpenNI2Grabber, program terminating!");
            THROW_ERROR_DESC( ERR_DEVICE_NOT_READY, "Unable to start Xtion Grabber")
        }
        // run thread worker
        m_oWorker.Run();
        }
    return cFilter::Start(__exception_ptr);
}

tResult cXtionCaptureFilter::Stop(ucom::IException** __exception_ptr)
{
    if (m_oWorker.GetState() == cKernelThread::TS_Running)
        {    
        m_oWorker.Suspend(tTrue);
        }
    // if grabber is running stop the grabber
    if (m_xtionGrabber.isRunning()) m_xtionGrabber.stop();    

    return cFilter::Stop(__exception_ptr);
}

tResult cXtionCaptureFilter::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    //terminate and release worker
    m_oWorker.Terminate(tTrue);
    m_oWorker.Release();
    
        return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cXtionCaptureFilter::ThreadFunc(cKernelThread* pThread, tVoid* pvUserData, tSize szUserData)
{
    if (pThread == &m_oWorker)
    {         
        //read data from xtion
        ReadData();
    }
    RETURN_NOERROR;
}

tResult cXtionCaptureFilter::ReadData()
{
    // check if state of thread and state of grabber is ok
    if ((GetState() != State_Running) || (m_xtionGrabber.isRunning()!= tTrue))
        RETURN_NOERROR;             
    
    if (m_oDepthImage.IsConnected())                    
    {   
        // create pointer to pixel image from xtion        
        openni::DepthPixel* ptrDepthImage;
        tInt DepthImageSize=0;
        // get pointer to data from xtion
        if(m_xtionGrabber.getDepthFramePtr(ptrDepthImage,DepthImageSize))
        {                
            //write date to opencv Mat
            cv::Mat depth(m_sBitmapFormatDepth.nWidth,m_sBitmapFormatDepth.nHeight,CV_16UC1,ptrDepthImage);
            // multiply the 11-bit depth values by 32 to extend the color range to a full 16-bits altough 16 bit is set in grabber            
            if (!m_setDepthInMillimeter) depth.convertTo(depth, -1, 32);                        
            // transmit data in media sample over the output pin
            cObjectPtr<IMediaSample> pNewDepthSample;
            if (IS_OK(AllocMediaSample(&pNewDepthSample)))
            {                    
                tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                pNewDepthSample->Update(tmStreamTime, depth.data, tInt32(DepthImageSize), 0);
                m_oDepthImage.Transmit(pNewDepthSample);
            }
        }
    }
    if (m_oRGBVideo.IsConnected())
    {
        // create pointer to pixel image from xtion            
        openni::RGB888Pixel* ptrColorImage;
        tInt colorImageSize=0;
        // get pointer to data from xtion            
        if (m_xtionGrabber.getColorFramePtr(ptrColorImage,colorImageSize))
        {
            //write date to opencv Mat
            cv::Mat colorImage(m_sBitmapFormat.nWidth,m_sBitmapFormat.nHeight,CV_8UC3,ptrColorImage);
            //convert from BGR to RGB            
            cv::cvtColor(colorImage, colorImage, CV_BGR2RGB);
            // transmit data in media sample over the output pin
            cObjectPtr<IMediaSample> pNewRGBSample;
            if (IS_OK(AllocMediaSample(&pNewRGBSample)))
            {                            
                tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                pNewRGBSample->Update(tmStreamTime, colorImage.data, tInt32(colorImageSize), 0);
                m_oRGBVideo.Transmit(pNewRGBSample);
            }    
        }
    }    
    RETURN_NOERROR;
}


tResult cXtionCaptureFilter::LoadConfiguration()
{
        //Get path of configuration file
    m_fileConfig = GetPropertyStr("Configuration File");
    // check if given property is not empty    
    if (m_fileConfig.IsEmpty())
    {
        LOG_WARNING("Xtion Device: Configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    //create path from path    
    ADTF_GET_CONFIG_FILENAME(m_fileConfig);
    m_fileConfig = m_fileConfig.CreateAbsolutePath(".");

    //Load file, parse configuration

    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);        
        cDOMElementRefList oElems;
        // load settings for the depthImage
        if(IS_OK(oDOM.FindNodes("xtionSettings/depthImage", oElems)))
        {                
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            { 
                cDOMElement* pConfigElement;
                if (IS_OK((*itElem)->FindNode("x_resolution", pConfigElement)))
                {                       
                    m_nWidthDepth = (pConfigElement->GetData()).AsInt();
                }
                if (IS_OK((*itElem)->FindNode("y_resolution", pConfigElement)))
                {
                    m_nHeightDepth = (pConfigElement->GetData()).AsInt();
                }
                if (IS_OK((*itElem)->FindNode("fps", pConfigElement)))
                {
                    m_FPSDepth = (pConfigElement->GetData()).AsInt();
                }
            }
        }
        // load settings for the colorImage
        if(IS_OK(oDOM.FindNodes("xtionSettings/colorImage", oElems)))
            {                
                 for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            { 
                cDOMElement* pConfigElement;
                if (IS_OK((*itElem)->FindNode("x_resolution", pConfigElement)))
                {                       
                    m_nWidthVideo = (pConfigElement->GetData()).AsInt();
                }
                if (IS_OK((*itElem)->FindNode("y_resolution", pConfigElement)))
                {
                    m_nHeightVideo = (pConfigElement->GetData()).AsInt();
                }
                if (IS_OK((*itElem)->FindNode("fps", pConfigElement)))
                {
                    m_FPSVideo = (pConfigElement->GetData()).AsInt();
                }
            }
        }
        // load setting for general configuration of xtion        
            if(IS_OK(oDOM.FindNodes("xtionSettings/options", oElems)))
            {                
                 for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            { 
                cDOMElement* pConfigElement;
                if (IS_OK((*itElem)->FindNode("setDepthColorSync", pConfigElement)))
                {                       
                    m_setDepthColorSync = (pConfigElement->GetData()).AsBool();
                }
                if (IS_OK((*itElem)->FindNode("setRegistration", pConfigElement)))
                {
                     m_setRegistration = (pConfigElement->GetData()).AsBool();
                } 
                if (IS_OK((*itElem)->FindNode("setDepthInMillimeter", pConfigElement)))
                {
                     m_setDepthInMillimeter = (pConfigElement->GetData()).AsBool();
                }                   
            }
        }
        //print result of load configuration to console
        if (oElems.size() > 0)
        {
            LOG_INFO("Xtion Device: Loaded configuration files successfully");
            LOG_INFO(cString::Format("Xtion Device: Video Format: %d x %d  @ %d fps",m_nWidthVideo,m_nHeightVideo,m_FPSVideo));
            LOG_INFO(cString::Format("Xtion Device: Depth Format: %d x %d  @ %d fps",m_nWidthDepth,m_nHeightDepth,m_FPSDepth));
            LOG_INFO(cString::Format("Xtion Device: Options: setDepthColorSync: %d, setRegistration: %d  ",m_setDepthColorSync,m_setRegistration));    
        }
    }
    else
    {
        LOG_ERROR("Xtion Device: Configured configuration file not found or could not be read");
        RETURN_ERROR(ERR_INVALID_FILE);
    }


    RETURN_NOERROR;
}

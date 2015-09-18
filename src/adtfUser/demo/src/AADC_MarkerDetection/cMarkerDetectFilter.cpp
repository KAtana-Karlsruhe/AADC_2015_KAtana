/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-01-30 08:17:46#$ $Rev:: 30939   $
**********************************************************************/

#include "stdafx.h"
#include "cMarkerDetectFilter.h"


ADTF_FILTER_PLUGIN("Marker Detection Filter Plugin", OID_ADTF_MARKERDETECTFILTER, cMarkerDetectFilter)



cMarkerDetectFilter::cMarkerDetectFilter(const tChar* __info):cFilter(__info)
{
    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::Start", m_oProcessStart);

    SetPropertyBool("Debug Output to Console", false);
    //initialize values for aruco detection    
    m_TheMarkerSize = -1;

    SetPropertyStr("Dictionary File For Markers",""); 
    SetPropertyBool("Dictionary File For Markers" NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr("Dictionary File For Markers" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)"); 

    
    SetPropertyInt("Video Output Pin",2);
    SetPropertyStr("Video Output Pin" NSSUBPROP_VALUELIST, "1@None|2@Erkannte Zeichen|");


        UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::End", m_oProcessEnd);    
}

cMarkerDetectFilter::~cMarkerDetectFilter()
{
}

tResult cMarkerDetectFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
    {
    //this is a VideoPin
    RETURN_IF_FAILED(m_oPinInputVideo.Create("Video_RGB_input",IPin::PD_Input, static_cast<IPinEventSink*>(this))); 
    RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));

    //this is a VideoPin
    RETURN_IF_FAILED(m_oPinOutputVideo.Create("Video_RGB_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_oPinOutputVideo));


    cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid**)&pDescManager,
                                             __exception_ptr));
    //this is a OutputPin
          tChar const * strDesc = pDescManager->GetMediaDescription("tRoadSign");
    if(!strDesc)
        LOG_ERROR(cString(OIGetInstanceName()) + ": Could not load mediadescription tRoadSign, check path");
        RETURN_IF_POINTER_NULL(strDesc);
        cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tRoadSign", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oPinRoadSign.Create("RoadSign", pType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSign));
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescRoadSign));

    }
    else if (eStage == StageNormal)
    {

    m_outputMode = GetPropertyInt("Video Output Pin");
    
    m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");

       //Get path of configuration file
      cFilename fileConfig = GetPropertyStr("Dictionary File For Markers");
        
        ADTF_GET_CONFIG_FILENAME(fileConfig);
       fileConfig = fileConfig.CreateAbsolutePath(".");
    
    if (fileConfig.IsEmpty() || !(cFileSystem::Exists(fileConfig)))
        {
        LOG_ERROR("Dictionary File For Markers not found");
        RETURN_ERROR(ERR_INVALID_FILE);
        }
    

        if(m_Dictionary.fromFile(string(fileConfig))==false) {
      LOG_ERROR("Dictionary File For Markers not found");
    };

        if(m_Dictionary.size()==0) {
      LOG_ERROR("Dictionary File For Markers not found");
    };
    
    HighlyReliableMarkers::loadDictionary(m_Dictionary);

    }
    else if (eStage == StageGraphReady)
    {
    cObjectPtr<IMediaType> pType;
    RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

    cObjectPtr<IMediaTypeVideo> pTypeVideo;
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));


    UpdateInputImageFormat(pTypeVideo->GetFormat());
    UpdateOutputImageFormat(pTypeVideo->GetFormat());  

    m_oPinInputVideo.SetFormat(&m_sInputFormat, NULL);  
    
    m_MDetector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
    m_MDetector.setThresholdParams( 21, 7);
    m_MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
    m_MDetector.setWarpSize((m_Dictionary[0].n()+2)*8);
    m_MDetector.setMinMaxSize(0.005, 0.5);

    }
    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cMarkerDetectFilter::OnPinEvent(IPin* pSource,
                                            tInt nEventCode,
                                            tInt nParam1,
                                            tInt nParam2,
                                            IMediaSample* pMediaSample)
{
    switch (nEventCode)
            {
            case IPinEventSink::PE_MediaSampleReceived:
                {
            if (pSource == &m_oPinInputVideo)
            {                
                UCOM_TIMING_SPOT(m_oProcessStart);
                ProcessVideo(pMediaSample);        
                UCOM_TIMING_SPOT(m_oProcessEnd);
            }                    
            break;
        }
        case IPinEventSink::PE_MediaTypeChanged:
        {
            if (pSource == &m_oPinInputVideo)
            {
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

                UpdateInputImageFormat(m_oPinInputVideo.GetFormat());    
                UpdateOutputImageFormat(m_oPinInputVideo.GetFormat());                
            }            
            break;
        }
        default:
        {
            break;
        }
        }    
    RETURN_NOERROR;
}



tResult cMarkerDetectFilter::ProcessVideo(adtf::IMediaSample* pISample)
{
    
    RETURN_IF_POINTER_NULL(pISample);

    //creating new media sample for output
    cObjectPtr<IMediaSample> pNewSample;
    RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
    RETURN_IF_FAILED(pNewSample->AllocBuffer(m_sOutputFormat.nSize));    
    
    //creating new pointer for input data
    const tVoid* l_pSrcBuffer;
    Mat TheInputImage;
    //receiving data from inputsample, and saving to inputFrame
    if (IS_OK(pISample->Lock(&l_pSrcBuffer)))
        {
        //convert to mat
        TheInputImage  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);    
        m_MDetector.detect(TheInputImage,m_TheMarkers,m_TheCameraParameters,m_TheMarkerSize);
        }
    pISample->Unlock(l_pSrcBuffer); 
    //print marker info and draw the markers in image    
    for (unsigned int i=0;i<m_TheMarkers.size();i++) 
        {            
        if (m_outputMode!=1) m_TheMarkers[i].draw(TheInputImage,Scalar(0,0,255),1);
        markerDetected(m_TheMarkers[i].id, m_TheMarkers[i].getArea(),pISample->GetTime());
        }

    //update new media sample with image data if something has ti be transmitted
    if (m_outputMode!=1) 
        {
        pNewSample->Update(pISample->GetTime(), TheInputImage.data, m_sOutputFormat.nSize, 0);
        m_oPinOutputVideo.Transmit(pNewSample);    
        }    
    // send a sample which indicates that no sign was detectec in this frame
    //if (!(m_SignInFrame)) sendRoadSignStruct(0,0,pISample->GetTime());
    
    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sInputFormat = (*pFormat);
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth,m_sInputFormat.nHeight,m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));        
        
    }
    
    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::UpdateOutputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sOutputFormat = (*pFormat);    
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, PixelFormat; %d", m_sOutputFormat.nWidth,m_sOutputFormat.nHeight,m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        
        m_oPinOutputVideo.SetFormat(&m_sOutputFormat, NULL);                
    }
    
    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::markerDetected(tInt16 markerID, tFloat32 MarkerSize, tTimeStamp timeOfFrame)
{
    switch (markerID)
        {
        case 491:
            //Vorfahrt gewaehren
            sendRoadSignStruct(1, MarkerSize, timeOfFrame); break;
        case 371:
            //Vorfahrt an naechster Kreuzung
            sendRoadSignStruct(2, MarkerSize, timeOfFrame); break;
        case 140:
            //Halt! Vorfahrt gewaehren (Stop)
            sendRoadSignStruct(3, MarkerSize, timeOfFrame); break;
        case 484:
            //Parken
            sendRoadSignStruct(4, MarkerSize, timeOfFrame); break;
        case 166:
            //Vorgeschriebene Fahrtrichtung geradeaus
            sendRoadSignStruct(5, MarkerSize, timeOfFrame); break;
        case 466:
            //Kreuzung
            sendRoadSignStruct(6, MarkerSize, timeOfFrame); break;
        case 376:
            //Fussgaengerueberweg
            sendRoadSignStruct(7, MarkerSize, timeOfFrame); break;
        case 46:
            //Kreisverkehr
            sendRoadSignStruct(8, MarkerSize, timeOfFrame); break;
        case 340:
            //Ueberholverbot
            sendRoadSignStruct(9, MarkerSize, timeOfFrame); break;
        case 306:
            //Verbot der Einfahrt
            sendRoadSignStruct(10, MarkerSize, timeOfFrame); break;
        case 82:
            //Einbahnstrasse
            sendRoadSignStruct(11, MarkerSize, timeOfFrame); break;
        default:
            break;
    
        }
    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::sendRoadSignStruct(tInt8 ID, tFloat32 MarkerSize, tTimeStamp timeOfFrame)
{    
    if (ID != 0) m_SignInFrame = tTrue;

    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescRoadSign->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

                {   // focus for sample write lock
                    //write date to the media sample with the coder of the descriptor
                    __adtf_sample_write_lock_mediadescription(m_pCoderDescRoadSign,pMediaSample,pCoder);

                    pCoder->Set("i8Identifier", (tVoid*)&ID);
                    pCoder->Set("fl32Imagesize", (tVoid*)&MarkerSize);       

                    pMediaSample->SetTime(timeOfFrame);
                }
    
    RETURN_IF_FAILED(m_oPinRoadSign.Transmit(pMediaSample));
    
    if (m_bDebugModeEnabled)  LOG_INFO(cString::Format("Zeichen ID %d erkannt. Area: %f",ID,MarkerSize));
    
    RETURN_NOERROR;
}


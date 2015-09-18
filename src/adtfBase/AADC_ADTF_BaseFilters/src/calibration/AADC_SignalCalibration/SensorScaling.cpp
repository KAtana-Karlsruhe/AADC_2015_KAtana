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
* $Author:: forchhe#$  $Date:: 2014-09-11 10:10:54#$ $Rev:: 25921   $
**********************************************************************/

// arduinofilter.cpp : Definiert die exportierten Funktionen für die DLL-Anwendung.
//
#include "stdafx.h"
#include "SensorScaling.h"

ADTF_FILTER_PLUGIN("AADC Calibration Scaling", OID_ADTF_SENSOR_SCALING, SensorScaling)

SensorScaling::SensorScaling(const tChar* __info) : cFilter(__info)
{
    m_ScaleFactor = 1.0;
    SetPropertyFloat("Scale Factor",m_ScaleFactor);
    SetPropertyFloat("Scale Factor" NSSUBPROP_REQUIRED, tTrue);
}

SensorScaling::~SensorScaling()
{
}

tResult SensorScaling::CreateInputPins(__exception)
{    
    
    RETURN_IF_FAILED(m_oInput.Create("input_value", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInput));
    RETURN_NOERROR;
}

tResult SensorScaling::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
    
    RETURN_IF_FAILED(m_oOutput.Create("output_value", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutput));

    RETURN_NOERROR;
}

tResult SensorScaling::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
        {
                CreateInputPins(__exception_ptr);
                CreateOutputPins(__exception_ptr);
        }
    m_ScaleFactor = tFloat32(GetPropertyFloat("Scale Factor"));            
    RETURN_NOERROR;
}

tResult SensorScaling::Start(__exception)
{
    
    return cFilter::Start(__exception_ptr);
}

tResult SensorScaling::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult SensorScaling::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult SensorScaling::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {        
        if (pMediaSample != NULL && m_pCoderDescSignal != NULL)
        {


            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;
            {   // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pCoderDescSignal,pMediaSample,pCoderInput);
               
                //get values from media sample        
                pCoderInput->Get("f32Value", (tVoid*)&value);
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            }
            // doing the calibration
            value = (m_ScaleFactor * value);
            //if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Sensorfilter received: ID %x Value %f",ID,value));   
                
            //create new media sample
            cObjectPtr<IMediaSample> pNewMediaSample;
            AllocMediaSample((tVoid**)&pNewMediaSample);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pNewMediaSample->AllocBuffer(nSize);
            {   // focus for sample write lock
                //write date to the media sample with the coder of the descriptor
                __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal,pNewMediaSample,pCoderOutput);
    
                pCoderOutput->Set("f32Value", (tVoid*)&(value));    
                pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            }    

            //transmit media sample over output pin
            pNewMediaSample->SetTime(pMediaSample->GetTime());
            m_oOutput.Transmit(pNewMediaSample);
        }
    }
    
    RETURN_NOERROR;
}


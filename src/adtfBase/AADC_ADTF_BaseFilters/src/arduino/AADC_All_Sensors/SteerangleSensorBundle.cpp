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

#include "stdafx.h"
#include "AbstractSensorBundle.h"
#include "SteerangleSensorBundle.h"

cSteerangleSensorBundle::cSteerangleSensorBundle(const tChar* __info) : cAbstractSensorsBundle(__info){

}

tResult cSteerangleSensorBundle::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));     
    
    RETURN_IF_FAILED(m_outputPin_steerAngle.Create("steerAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_steerAngle));    
    RETURN_NOERROR;
}

tResult cSteerangleSensorBundle::CreateInputPins(__exception){
    RETURN_IF_FAILED(m_oInput.Create("steering_servo", new cMediaType(0, 0, 0, "tSteeringAngleData"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInput));
    RETURN_NOERROR;
}

tResult cSteerangleSensorBundle::PrintSamplingRate(tFloat64 median, tFloat64 variance)
{
    LOG_INFO(cString::Format("steering_servo Samplerate Mittelwert: %f Hz, Varianz: %f Hz",median,variance));
    RETURN_NOERROR;          
}

tResult cSteerangleSensorBundle::ProcessData(IMediaSample* pMediaSample)
{
    if (pMediaSample != NULL && m_pCoderDescInput != NULL)
    {
         tUInt16 angle = 0;
        tUInt32 timestampValue = 0;              
        // read-out the incoming Media Sample
        {   // focus for sample read lock
            //LOG_INFO("Received");
            __adtf_sample_read_lock_mediadescription(m_pCoderDescInput,pMediaSample,pCoder);
       
            pCoder->Get("ui16Angle", (tVoid*)&angle);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timestampValue);  
        }
                
        tFloat32 flAngle = tFloat32(angle);
        
        TransmitData(pMediaSample->GetTime(),flAngle, timestampValue);
    }
    
    RETURN_NOERROR;  
}

tResult cSteerangleSensorBundle::TransmitData(tTimeStamp inputTimeStamp, tFloat32 steerAngle, tUInt32 timestampValue)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;    
    AllocMediaSample((tVoid**)&pMediaSample);
    
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);          
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSample,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&(steerAngle));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSample->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_steerAngle.Transmit(pMediaSample);
    
    RETURN_NOERROR;  
 }

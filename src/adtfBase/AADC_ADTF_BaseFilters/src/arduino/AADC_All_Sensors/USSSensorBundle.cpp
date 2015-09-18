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
#include "USSSensorBundle.h"

cUSSSensorBundle::cUSSSensorBundle(const tChar* __info) : cAbstractSensorsBundle(__info){

}

tResult cUSSSensorBundle::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));     
    
    RETURN_IF_FAILED(m_outputPin_front_left.Create("range_front_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_front_left));
    
    RETURN_IF_FAILED(m_outputPin_front_right.Create("range_front_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_front_right));

    RETURN_IF_FAILED(m_outputPin_rear_left.Create("range_rear_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_rear_left));

    RETURN_IF_FAILED(m_outputPin_rear_right.Create("range_rear_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_rear_right));
    
    RETURN_NOERROR;
}

tResult cUSSSensorBundle::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oInput.Create("ultrasonic_sensors", new cMediaType(0, 0, 0, "tUsData"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInput));

    RETURN_NOERROR;
}

tResult cUSSSensorBundle::PrintSamplingRate(tFloat64 median, tFloat64 variance)
{
    LOG_INFO(cString::Format("Ultrasonic Sensor Samplerate Mittelwert: %f Hz, Varianz: %f Hz",median,variance));
    RETURN_NOERROR;          
}

tResult cUSSSensorBundle::ProcessData(IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    if (pMediaSample != NULL && m_pCoderDescInput != NULL)
    {
        
        //write values with zero
        tUInt16 front_Left = 0;
        tUInt16 front_Right = 0;
        tUInt16 rear_Left = 0;
        tUInt16 rear_Right = 0;  
        tUInt32 timestampValue = 0;          
        
        //get values from media sample
        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescInput,pMediaSample,pCoder);
        
            pCoder->Get("ui16Front_Left", (tVoid*)&front_Left);
            pCoder->Get("ui16Front_Right", (tVoid*)&front_Right);
            pCoder->Get("ui16Rear_Left", (tVoid*)&rear_Left);
            pCoder->Get("ui16Rear_Right", (tVoid*)&rear_Right);          
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timestampValue);    
        }     

        //create and transmit output mediasamples
        tFloat32 flFront_Left = tFloat32(front_Left);
        tFloat32 flFront_Right = tFloat32(front_Right);
        tFloat32 flRear_Left = tFloat32(rear_Left);
        tFloat32 flRear_Right= tFloat32(rear_Right); 
        
        TransmitData(pMediaSample->GetTime(),flFront_Left,flFront_Right,flRear_Left,flRear_Right,timestampValue);       
    }
        
    RETURN_NOERROR;  
}

tResult cUSSSensorBundle::TransmitData(tTimeStamp inputTimeStamp, tFloat32 front_left,tFloat32 front_right, tFloat32 rear_left, tFloat32 rear_right, tUInt32 timestampValue)
{   
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleFrontLeft;
    cObjectPtr<IMediaSample> pMediaSampleFrontRight;
    cObjectPtr<IMediaSample> pMediaSampleRearLeft;
    cObjectPtr<IMediaSample> pMediaSampleRearRight;
    AllocMediaSample((tVoid**)&pMediaSampleFrontLeft);
    AllocMediaSample((tVoid**)&pMediaSampleFrontRight);
    AllocMediaSample((tVoid**)&pMediaSampleRearLeft);
    AllocMediaSample((tVoid**)&pMediaSampleRearRight);  
    

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleFrontLeft->AllocBuffer(nSize);
    pMediaSampleFrontRight->AllocBuffer(nSize);
    pMediaSampleRearLeft->AllocBuffer(nSize);
    pMediaSampleRearRight->AllocBuffer(nSize);         
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleFrontLeft,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&front_left);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSampleFrontLeft->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_front_left.Transmit(pMediaSampleFrontLeft);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleFrontRight,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&front_right);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSampleFrontRight->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_front_right.Transmit(pMediaSampleFrontRight);
    {   // focus for sample write lock    
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleRearLeft,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&rear_left);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);    
    }    
    pMediaSampleRearLeft->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_rear_left.Transmit(pMediaSampleRearLeft);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleRearRight,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&rear_right);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSampleRearRight->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_rear_right.Transmit(pMediaSampleRearRight);
    
    
    RETURN_NOERROR;
}

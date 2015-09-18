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
#include "GyroSensorBundle.h"

cGyroSensorBundle::cGyroSensorBundle(const tChar* __info) : cAbstractSensorsBundle(__info){

}

tResult cGyroSensorBundle::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));     
 
    RETURN_IF_FAILED(m_outputPin_gyro_w.Create("quaternion_gyro_w", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_gyro_w));
    
    RETURN_IF_FAILED(m_outputPin_gyro_x.Create("quaternion_gyro_x", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_gyro_x));

    RETURN_IF_FAILED(m_outputPin_gyro_y.Create("quaternion_gyro_y", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_gyro_y));

    RETURN_IF_FAILED(m_outputPin_gyro_z.Create("quaternion_gyro_z", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_gyro_z));
    RETURN_NOERROR;
}

tResult cGyroSensorBundle::CreateInputPins(__exception){
    RETURN_IF_FAILED(m_oInput.Create("gyroscope", new cMediaType(0, 0, 0, "tGyroData"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInput));

    RETURN_NOERROR;
}


tResult cGyroSensorBundle::PrintSamplingRate(tFloat64 median, tFloat64 variance)
{
    LOG_INFO(cString::Format("Gyrometer Samplerate Mittelwert: %f Hz, Varianz: %f Hz",median,variance));  
    RETURN_NOERROR;        
}

tResult cGyroSensorBundle::ProcessData(IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    if (pMediaSample != NULL && m_pCoderDescInput != NULL)
    {
        //write values with zero
        tInt16 q_w = 0;
        tInt16 q_x = 0;
        tInt16 q_y = 0;
        tInt16 q_z = 0;
        tUInt32 timestampValue = 0;

        {   // focus for sample read lock          
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescInput,pMediaSample,pCoder);
            //get values from media sample
            pCoder->Get("i16Q_w", (tVoid*)&q_w);
            pCoder->Get("i16Q_x", (tVoid*)&q_x);
            pCoder->Get("i16Q_y", (tVoid*)&q_y);
            pCoder->Get("i16Q_z", (tVoid*)&q_z);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timestampValue);         
        }
        
        //create and transmit output mediasamples and normalize them to 16384.0f (maximum value from raw value from sensor)
        tFloat32 flQ_w =tFloat32(q_w)/16384.0f;
        tFloat32 flQ_x =tFloat32(q_x)/16384.0f;
        tFloat32 flQ_y =tFloat32(q_y)/16384.0f;
        tFloat32 flQ_z =tFloat32(q_z)/16384.0f;
             
        TransmitData(pMediaSample->GetTime(),flQ_w,flQ_x,flQ_y,flQ_z,timestampValue);
              
    }
    RETURN_NOERROR;  
}

tResult cGyroSensorBundle::TransmitData(tTimeStamp inputTimeStamp, tFloat32 gyro_w, tFloat32 gyro_x,tFloat32 gyro_y, tFloat32 gyro_z,tUInt32 timestampValue)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleGyro_w;
    cObjectPtr<IMediaSample> pMediaSampleGyro_x;
    cObjectPtr<IMediaSample> pMediaSampleGyro_y;
    cObjectPtr<IMediaSample> pMediaSampleGyro_z;
    AllocMediaSample((tVoid**)&pMediaSampleGyro_w);
    AllocMediaSample((tVoid**)&pMediaSampleGyro_x);
    AllocMediaSample((tVoid**)&pMediaSampleGyro_y);
    AllocMediaSample((tVoid**)&pMediaSampleGyro_z);  
    

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleGyro_w->AllocBuffer(nSize);
    pMediaSampleGyro_x->AllocBuffer(nSize);
    pMediaSampleGyro_y->AllocBuffer(nSize);
    pMediaSampleGyro_z->AllocBuffer(nSize);         
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleGyro_w,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&(gyro_w));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSampleGyro_w->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_gyro_w.Transmit(pMediaSampleGyro_w);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleGyro_x,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&(gyro_x));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSampleGyro_x->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_gyro_x.Transmit(pMediaSampleGyro_x);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleGyro_y,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&(gyro_y));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSampleGyro_y->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_gyro_y.Transmit(pMediaSampleGyro_y);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleGyro_z,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&(gyro_z));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }        
    pMediaSampleGyro_z->SetTime(inputTimeStamp);
    //transmit media sample over output pin
    m_outputPin_gyro_z.Transmit(pMediaSampleGyro_z);
    
    RETURN_NOERROR;  
}

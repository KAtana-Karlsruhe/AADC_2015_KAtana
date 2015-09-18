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
#include "AccelSensorBundle.h"

cAccelSensorBundle::cAccelSensorBundle(const tChar* __info) : cAbstractSensorsBundle(__info){

}

tResult cAccelSensorBundle::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));     
 
    RETURN_IF_FAILED(m_outputPin_acc_x.Create("acc_x", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_acc_x));
    
    RETURN_IF_FAILED(m_outputPin_acc_y.Create("acc_y", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_acc_y));

    RETURN_IF_FAILED(m_outputPin_acc_z.Create("acc_z", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_acc_z));
    
    RETURN_IF_FAILED(m_outputPin_temperature.Create("temperature", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_temperature));

    RETURN_NOERROR;
}

tResult cAccelSensorBundle::CreateInputPins(__exception){
    RETURN_IF_FAILED(m_oInput.Create("accelerometer", new cMediaType(0, 0, 0, "tAccData"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInput));

    RETURN_NOERROR;
}

tResult cAccelSensorBundle::PrintSamplingRate(tFloat64 median, tFloat64 variance)
{
    LOG_INFO(cString::Format("Accelerometer Samplerate Mittelwert: %f Hz, Varianz: %f Hz",median,variance)); 
    RETURN_NOERROR;         
}

tResult cAccelSensorBundle::ProcessData(IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    if (pMediaSample != NULL && m_pCoderDescInput != NULL)
    {
        //write values with zero
        tInt16 a_x = 0;
        tInt16 a_y = 0;
        tInt16 a_z = 0;
        tInt16 temperature = 0;
        tUInt32 timestampValue = 0; 
        
        //get values from media sample
        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescInput,pMediaSample,pCoder);
            pCoder->Get("i16A_x", (tVoid*)&a_x);
            pCoder->Get("i16A_y", (tVoid*)&a_y);
            pCoder->Get("i16A_z", (tVoid*)&a_z);
            pCoder->Get("i16Temperature", (tVoid*)&temperature);     
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timestampValue);      
        }   
            
        //create and transmit output mediasamples
        tFloat32 flA_x =tFloat32(a_x);
        tFloat32 flA_y =tFloat32(a_y);
        tFloat32 flA_z =tFloat32(a_z);
        tFloat32 flTemperature =tFloat32(temperature);
        TransmitData(pMediaSample->GetTime(),flA_x,flA_y,flA_z,flTemperature,timestampValue); 
    }
        
    RETURN_NOERROR;  
}

tResult cAccelSensorBundle::TransmitData(tTimeStamp inputTimeStamp, tFloat32 flA_x, tFloat32 flA_y, tFloat32 flA_z,tFloat32 flTemperature, tUInt32 timestampValue)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleA_x;
    cObjectPtr<IMediaSample> pMediaSampleA_y;
    cObjectPtr<IMediaSample> pMediaSampleA_z;
    cObjectPtr<IMediaSample> pMediaSampleTemp;
    AllocMediaSample((tVoid**)&pMediaSampleA_x);
    AllocMediaSample((tVoid**)&pMediaSampleA_y);
    AllocMediaSample((tVoid**)&pMediaSampleA_z);
    AllocMediaSample((tVoid**)&pMediaSampleTemp);     

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleA_x->AllocBuffer(nSize);
    pMediaSampleA_y->AllocBuffer(nSize);
    pMediaSampleA_z->AllocBuffer(nSize);    
    pMediaSampleTemp->AllocBuffer(nSize);       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor 
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleA_x,pCoderOutputX);
        pCoderOutputX->Set("f32Value", (tVoid*)&(flA_x));    
        pCoderOutputX->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);  
    }    
    pMediaSampleA_x->SetTime(inputTimeStamp);  
    //transmit media sample over output pin
    m_outputPin_acc_x.Transmit(pMediaSampleA_x);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleA_y,pCoderOutputY);
        pCoderOutputY->Set("f32Value", (tVoid*)&(flA_y));    
        pCoderOutputY->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue); 
    }    
    pMediaSampleA_y->SetTime(inputTimeStamp);   
    //transmit media sample over output pin
    m_outputPin_acc_y.Transmit(pMediaSampleA_y);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleA_z,pCoderOutputZ);
        pCoderOutputZ->Set("f32Value", (tVoid*)&(flA_z));    
        pCoderOutputZ->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue); 
    }    
    pMediaSampleA_z->SetTime(inputTimeStamp);   
    //transmit media sample over output pin
    m_outputPin_acc_z.Transmit(pMediaSampleA_z);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
         __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleTemp,pCoderOutputTemp);
        pCoderOutputTemp->Set("f32Value", (tVoid*)&(flTemperature));    
        pCoderOutputTemp->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSampleTemp->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_temperature.Transmit(pMediaSampleTemp);
    
    RETURN_NOERROR;  
}

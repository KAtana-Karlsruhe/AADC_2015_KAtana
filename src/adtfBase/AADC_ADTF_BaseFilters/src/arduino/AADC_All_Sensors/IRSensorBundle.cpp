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
#include "IRSensorBundle.h"

cIRSensorBundle::cIRSensorBundle(const tChar* __info) : cAbstractSensorsBundle(__info){

}

tResult cIRSensorBundle::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));      

    //front center
    m_outputPin_front_center_longrange.Create("voltage_front_center_longrange",  pTypeSignalValue, static_cast<IPinEventSink*> (this));
    RegisterPin(&m_outputPin_front_center_longrange);        
    m_outputPin_front_center_shortrange.Create("voltage_front_center_shortrange",  pTypeSignalValue, static_cast<IPinEventSink*> (this));
    RegisterPin(&m_outputPin_front_center_shortrange);
    
    //front left
    m_outputPin_front_left_longrange.Create("voltage_front_left_longrange",  pTypeSignalValue, static_cast<IPinEventSink*> (this));
    RegisterPin(&m_outputPin_front_left_longrange);    
    m_outputPin_front_left_shortrange.Create("voltage_front_left_shortrange",  pTypeSignalValue, static_cast<IPinEventSink*> (this));
    RegisterPin(&m_outputPin_front_left_shortrange);
    
    //front right
    m_outputPin_front_right_longrange.Create("voltage_front_right_longrange", pTypeSignalValue, static_cast<IPinEventSink*> (this));
    RegisterPin(&m_outputPin_front_right_longrange);    
    m_outputPin_front_right_shortrange.Create("voltage_front_right_shortrange", pTypeSignalValue, static_cast<IPinEventSink*> (this));
    RegisterPin(&m_outputPin_front_right_shortrange);
    
    //rear center    
    m_outputPin_rear_center_shortrange.Create("voltage_rear_center_shortrange", pTypeSignalValue, static_cast<IPinEventSink*> (this));
    RegisterPin(&m_outputPin_rear_center_shortrange);
    
    //rear left    
    m_outputPin_rear_left_shortrange.Create("voltage_rear_left_shortrange", pTypeSignalValue, static_cast<IPinEventSink*> (this));
    RegisterPin(&m_outputPin_rear_left_shortrange);
    
    //rear right
    m_outputPin_rear_right_shortrange.Create("voltage_rear_right_shortrange", pTypeSignalValue, static_cast<IPinEventSink*> (this));
    RegisterPin(&m_outputPin_rear_right_shortrange);

    RETURN_NOERROR;
}

tResult cIRSensorBundle::CreateInputPins(__exception){
    RETURN_IF_FAILED(m_oInput.Create("infrared_sensors", new cMediaType(0, 0, 0, "tIrData"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInput));

    RETURN_NOERROR;
}

tResult cIRSensorBundle::PrintSamplingRate(tFloat64 median, tFloat64 variance)
{
    LOG_INFO(cString::Format("Infrared Sensor Samplerate Mittelwert: %f Hz, Varianz: %f Hz",median,variance));
    RETURN_NOERROR;          
}


tResult cIRSensorBundle::ProcessData(IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    if (pMediaSample != NULL && m_pCoderDescInput != NULL)
    {
        //write values with zero
        tUInt16 front_Center_Longrange = 0; 
        tUInt16 front_Center_Shortrange = 0; 
        tUInt16 front_Left_Longrange = 0; 
        tUInt16 front_Left_Shortrange = 0; 
        tUInt16 front_Right_Shortrange = 0; 
        tUInt16 front_Right_Longrange = 0; 
        tUInt16 rear_Center_Shortrange = 0; 
        tUInt16 rear_Left_Shortrange = 0; 
        tUInt16 rear_Right_Shortrange = 0;
        tUInt32 timestampValue = 0;          
        
        //get values from media sample
        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescInput,pMediaSample,pCoder);

            pCoder->Get("ui16Front_Center_Longrange", (tVoid*)&front_Center_Longrange);
            pCoder->Get("ui16Front_Center_Shortrange", (tVoid*)&front_Center_Shortrange);    
            pCoder->Get("ui16Front_Left_Longrange", (tVoid*)&front_Left_Longrange);    
            pCoder->Get("ui16Front_Left_Shortrange", (tVoid*)&front_Left_Shortrange);    
            pCoder->Get("ui16Front_Right_Shortrange", (tVoid*)&front_Right_Shortrange);    
            pCoder->Get("ui16Front_Right_Longrange", (tVoid*)&front_Right_Longrange);    
            pCoder->Get("ui16Rear_Center_Shortrange", (tVoid*)&rear_Center_Shortrange);    
            pCoder->Get("ui16Rear_Left_Shortrange", (tVoid*)&rear_Left_Shortrange);    
            pCoder->Get("ui16Rear_Right_Shortrange", (tVoid*)&rear_Right_Shortrange);     
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timestampValue);         
        }

        //create and transmit output mediasamples
        tFloat32 flFront_Center_Longrange = tFloat32(front_Center_Longrange); 
        tFloat32 flFront_Center_Shortrange = tFloat32(front_Center_Shortrange); 
        tFloat32 flFront_Left_Longrange = tFloat32(front_Left_Longrange); 
        tFloat32 flFront_Left_Shortrange = tFloat32(front_Left_Shortrange); 
        tFloat32 flFront_Right_Shortrange = tFloat32(front_Right_Shortrange); 
        tFloat32 flFront_Right_Longrange = tFloat32(front_Right_Longrange); 
        tFloat32 flRear_Center_Shortrange = tFloat32(rear_Center_Shortrange); 
        tFloat32 flRear_Left_Shortrange = tFloat32(rear_Left_Shortrange); 
        tFloat32 flRear_Right_Shortrange = tFloat32(rear_Right_Shortrange);
        
        TransmitData(pMediaSample->GetTime(),flFront_Center_Longrange, flFront_Center_Shortrange, flFront_Left_Longrange, flFront_Left_Shortrange, flFront_Right_Longrange, flFront_Right_Shortrange, flRear_Center_Shortrange, flRear_Left_Shortrange, flRear_Right_Shortrange, timestampValue);
              
              
    }
        
    RETURN_NOERROR;  
}

tResult cIRSensorBundle::TransmitData(tTimeStamp inputTimeStamp, tFloat32 front_center_longrange, tFloat32 front_center_shortrange, tFloat32 front_left_longrange, tFloat32 front_left_shortrange, tFloat32 front_right_longrange, tFloat32 front_right_shortrange, tFloat32 rear_center_shortrange, tFloat32 rear_left_shortrange, tFloat32 rear_right_shortrange, tUInt32 timestampValue)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleFrontCenterLong;
    cObjectPtr<IMediaSample> pMediaSampleFrontCenterShort;
    cObjectPtr<IMediaSample> pMediaSampleFrontLeftLong;
    cObjectPtr<IMediaSample> pMediaSampleFrontLeftShort;
    cObjectPtr<IMediaSample> pMediaSampleFrontRightLong;
    cObjectPtr<IMediaSample> pMediaSampleFrontRightShort;
    cObjectPtr<IMediaSample> pMediaSampleRearCenterShort;
    cObjectPtr<IMediaSample> pMediaSampleRearLeftShort;
    cObjectPtr<IMediaSample> pMediaSampleRearRightShort;
    
    AllocMediaSample((tVoid**)&pMediaSampleFrontCenterLong);
    AllocMediaSample((tVoid**)&pMediaSampleFrontCenterShort);
    AllocMediaSample((tVoid**)&pMediaSampleFrontLeftLong);
    AllocMediaSample((tVoid**)&pMediaSampleFrontLeftShort);
    AllocMediaSample((tVoid**)&pMediaSampleFrontRightLong);
    AllocMediaSample((tVoid**)&pMediaSampleFrontRightShort);
    AllocMediaSample((tVoid**)&pMediaSampleRearCenterShort);
    AllocMediaSample((tVoid**)&pMediaSampleRearLeftShort);
    AllocMediaSample((tVoid**)&pMediaSampleRearRightShort);    

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleFrontCenterLong->AllocBuffer(nSize);
    pMediaSampleFrontCenterShort->AllocBuffer(nSize); 
    pMediaSampleFrontLeftLong->AllocBuffer(nSize); 
    pMediaSampleFrontLeftShort->AllocBuffer(nSize); 
    pMediaSampleFrontRightLong->AllocBuffer(nSize); 
    pMediaSampleFrontRightShort->AllocBuffer(nSize); 
    pMediaSampleRearCenterShort->AllocBuffer(nSize); 
    pMediaSampleRearLeftShort->AllocBuffer(nSize);
    pMediaSampleRearRightShort->AllocBuffer(nSize);      
    
    
    cObjectPtr<IMediaCoder> pCoderOutput;   
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor            
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleFrontCenterLong,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&front_center_longrange);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);   
    }    
    pMediaSampleFrontCenterLong->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_front_center_longrange.Transmit(pMediaSampleFrontCenterLong);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleFrontCenterShort,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&front_center_shortrange);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);   
    }    
    pMediaSampleFrontCenterShort->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_front_center_shortrange.Transmit(pMediaSampleFrontCenterShort);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleFrontLeftLong,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&front_left_longrange);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);   
    }    
    pMediaSampleFrontLeftLong->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_front_left_longrange.Transmit(pMediaSampleFrontLeftLong);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleFrontLeftShort,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&front_left_shortrange);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);   
    }    
    pMediaSampleFrontLeftShort->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_front_left_shortrange.Transmit(pMediaSampleFrontLeftShort);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleFrontRightLong,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&front_right_longrange);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);   
    }    
    pMediaSampleFrontRightLong->SetTime(inputTimeStamp);    
    //transmit media sample over output pin
    m_outputPin_front_right_longrange.Transmit(pMediaSampleFrontRightLong);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleFrontRightShort,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&front_right_shortrange);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);  
    }    
    pMediaSampleFrontRightShort->SetTime(inputTimeStamp);     
    //transmit media sample over output pin
    m_outputPin_front_right_shortrange.Transmit(pMediaSampleFrontRightShort);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleRearCenterShort,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&rear_center_shortrange);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue); 
    }    
    pMediaSampleRearCenterShort->SetTime(inputTimeStamp);      
    //transmit media sample over output pin
    m_outputPin_rear_center_shortrange.Transmit(pMediaSampleRearCenterShort);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleRearLeftShort,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&rear_left_shortrange);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSampleRearLeftShort->SetTime(inputTimeStamp);       
    //transmit media sample over output pin
    m_outputPin_rear_left_shortrange.Transmit(pMediaSampleRearLeftShort);
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput,pMediaSampleRearRightShort,pCoderOutput);
        pCoderOutput->Set("f32Value", (tVoid*)&rear_right_shortrange);    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timestampValue);
    }    
    pMediaSampleRearRightShort->SetTime(inputTimeStamp);       
    //transmit media sample over output pin
    m_outputPin_rear_right_shortrange.Transmit(pMediaSampleRearRightShort);

    RETURN_NOERROR;
}

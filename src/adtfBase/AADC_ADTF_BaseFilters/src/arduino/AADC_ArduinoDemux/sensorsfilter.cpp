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
#include "sensorsfilter.h"


ADTF_FILTER_PLUGIN("AADC Sensors", OID_ADTF_ARDUINO_FILTER, cArduinoSensors)

cArduinoSensors::cArduinoSensors(const tChar* __info) : cFilter(__info)
{    
    m_bDebugModeEnabled = tFalse;
    SetPropertyBool("DebugOutput",              m_bDebugModeEnabled);
}

cArduinoSensors::~cArduinoSensors()
{
}

//****************************
tResult cArduinoSensors::CreateInputPins(){
    RETURN_IF_FAILED(m_oArduinoCOMInputPin.Create("ArduinoCOM_input", new cMediaType(0, 0, 0, "tArduinoData"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oArduinoCOMInputPin));
    RETURN_NOERROR;
}

//*****************************************************************************

tResult cArduinoSensors::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
        
    // USS Output
    tChar const * strDescUs = pDescManager->GetMediaDescription("tUsData");
    RETURN_IF_POINTER_NULL(strDescUs);
    cObjectPtr<IMediaType> pTypeUsData = new cMediaType(0, 0, 0, "tUsData", strDescUs,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(m_oUSSOutputPin.Create("ultrasonic_sensors", pTypeUsData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oUSSOutputPin));
    RETURN_IF_FAILED(pTypeUsData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescUsData)); 
    
    // Ir Output and descriptor
    tChar const * strDescIr = pDescManager->GetMediaDescription("tIrData");
    RETURN_IF_POINTER_NULL(strDescIr);
    cObjectPtr<IMediaType> pTypeIrData = new cMediaType(0, 0, 0, "tIrData", strDescIr,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(m_oIROutputPin.Create("infrared_sensors", pTypeIrData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oIROutputPin));
    RETURN_IF_FAILED(pTypeIrData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescIrData)); 

    
    //steering_servo output and descriptor
    tChar const * strDescSteering = pDescManager->GetMediaDescription("tSteeringAngleData");
    RETURN_IF_POINTER_NULL(strDescSteering);
    cObjectPtr<IMediaType> pTypeSteeringData = new cMediaType(0, 0, 0, "tSteeringAngleData", strDescSteering,IMediaDescription::MDF_DDL_DEFAULT_VERSION);        
    RETURN_IF_FAILED(m_oSteeringAngleOutputPin.Create("steering_servo", pTypeSteeringData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngleOutputPin));
    RETURN_IF_FAILED(pTypeSteeringData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSteeringAngleData)); 
    
    
    // ACC Output and descriptor
    tChar const * strDescAcc = pDescManager->GetMediaDescription("tAccData");
    RETURN_IF_POINTER_NULL(strDescAcc);
    cObjectPtr<IMediaType> pTypeAccData = new cMediaType(0, 0, 0, "tAccData", strDescAcc,IMediaDescription::MDF_DDL_DEFAULT_VERSION);        
    RETURN_IF_FAILED(m_oACCOutputPin.Create("accelerometer", pTypeAccData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oACCOutputPin));
    RETURN_IF_FAILED(pTypeAccData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescAccData)); 
    
    // Gyro Output and descriptor
    tChar const * strDescGyro = pDescManager->GetMediaDescription("tGyroData");
    RETURN_IF_POINTER_NULL(strDescGyro);
    cObjectPtr<IMediaType> pTypeGyroData = new cMediaType(0, 0, 0, "tGyroData", strDescGyro,IMediaDescription::MDF_DDL_DEFAULT_VERSION);        
    RETURN_IF_FAILED(m_oGyroOutputPin.Create("gyroscope", pTypeGyroData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oGyroOutputPin));
    RETURN_IF_FAILED(pTypeGyroData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescGyroData)); 
    
    // Wheel Output and descriptor
    tChar const * strDescWheelEncoder = pDescManager->GetMediaDescription("tWheelEncoderData");
    RETURN_IF_POINTER_NULL(strDescWheelEncoder);
    cObjectPtr<IMediaType> pTypeWheelEncoderData = new cMediaType(0, 0, 0, "tWheelEncoderData", strDescWheelEncoder,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(m_oWheelCounterOutputPin.Create("wheel_speed_sensor", pTypeWheelEncoderData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oWheelCounterOutputPin));
    RETURN_IF_FAILED(pTypeWheelEncoderData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescWheelEncoderData)); 

    //Voltage Output and descriptor
    tChar const * strDescVoltage = pDescManager->GetMediaDescription("tVoltageData");
    RETURN_IF_POINTER_NULL(strDescVoltage);
    cObjectPtr<IMediaType> pTypeVoltageData = new cMediaType(0, 0, 0, "tVoltageData", strDescVoltage,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(m_oVoltOutputPin.Create("system_voltage", pTypeVoltageData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oVoltOutputPin));
    RETURN_IF_FAILED(pTypeVoltageData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescVoltageData)); 
   
    RETURN_NOERROR;
}

//*****************************************************************************

tResult cArduinoSensors::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        tResult nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult))
        {
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");
        }
     nResult = CreateInputPins();
        if (IS_FAILED(nResult))
        {
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");
        }        
    }
    else if (eStage == StageNormal)
    {
      
        m_bDebugModeEnabled = GetPropertyBool("DebugOutput");  
    }
    RETURN_NOERROR;
}

tResult cArduinoSensors::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cArduinoSensors::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cArduinoSensors::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cArduinoSensors::TransmitSteeringAngleData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData)
{
    //cast to struct
    tSteeringAngleData *arduinoData =  (tSteeringAngleData*) frameData;    
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSteeringAngleData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSteeringAngleData,pMediaSample,pCoder);

        pCoder->Set("ui16Angle", (tVoid*)&(arduinoData->ui16Angle));    
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&arduinoTimestamp);
    }    
    
    //transmit media sample over output pin
    pMediaSample->SetTime(inputTimeStamp);
    RETURN_IF_FAILED(m_oSteeringAngleOutputPin.Transmit(pMediaSample));
    RETURN_NOERROR;   
    
}

tResult cArduinoSensors::TransmitGyroData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData)
{
    //cast to struct
    tImuData *arduinoData =  (tImuData*) frameData;    
   
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescGyroData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescGyroData,pMediaSample,pCoder);
        pCoder->Set("i16Q_w", (tVoid*)&(arduinoData->i16Q_w));    
        pCoder->Set("i16Q_x", (tVoid*)&(arduinoData->i16Q_x));
        pCoder->Set("i16Q_y", (tVoid*)&(arduinoData->i16Q_y));
        pCoder->Set("i16Q_z", (tVoid*)&(arduinoData->i16Q_z));
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&arduinoTimestamp);
    }    
    
    //transmit media sample over output pin
    pMediaSample->SetTime(inputTimeStamp);
    RETURN_IF_FAILED(m_oGyroOutputPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult cArduinoSensors::TransmitAccData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData)
{   
    //cast to struct
    tImuData *arduinoData =  (tImuData*) frameData;    
     
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescAccData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescAccData,pMediaSample,pCoder);
        pCoder->Set("i16A_x", (tVoid*)&(arduinoData->i16A_x));
        pCoder->Set("i16A_y", (tVoid*)&(arduinoData->i16A_y));
        pCoder->Set("i16A_z", (tVoid*)&(arduinoData->i16A_z));
        pCoder->Set("i16Temperature", (tVoid*)&(arduinoData->i16A_x));     
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&arduinoTimestamp);
    }    
    
    //transmit media sample over output pin
    pMediaSample->SetTime(inputTimeStamp);
    RETURN_IF_FAILED(m_oACCOutputPin.Transmit(pMediaSample));
    
    RETURN_NOERROR;
}

tResult cArduinoSensors::TransmitUsData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData)
{   
    //cast to struct
    tUsData *arduinoData =  (tUsData*) frameData;    
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescUsData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescUsData,pMediaSample,pCoder);

        
        pCoder->Set("ui16Front_Left", (tVoid*)&(arduinoData->ui16FrontLeft));
        pCoder->Set("ui16Front_Right", (tVoid*)&(arduinoData->ui16FrontRight));
        pCoder->Set("ui16Rear_Left", (tVoid*)&(arduinoData->ui16RearLeft));
        pCoder->Set("ui16Rear_Right", (tVoid*)&(arduinoData->ui16RearRight));     
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&arduinoTimestamp);
    }    
    
    //transmit media sample over output pin
    pMediaSample->SetTime(inputTimeStamp);
    RETURN_IF_FAILED(m_oUSSOutputPin.Transmit(pMediaSample));    

    
    RETURN_NOERROR;
}

tResult cArduinoSensors::TransmitCountRPMData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData)
{       
    //cast to struct
    tWheelEncoderData *arduinoData =  (tWheelEncoderData*) frameData;    
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescWheelEncoderData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescWheelEncoderData,pMediaSample,pCoder);

        pCoder->Set("ui32LeftWheel", (tVoid*)&(arduinoData->ui32LeftWheel));
        pCoder->Set("ui32RightWheel", (tVoid*)&(arduinoData->ui32RightWheel));    
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&arduinoTimestamp);
    }    
    
    //transmit media sample over output pin
    pMediaSample->SetTime(inputTimeStamp);
    RETURN_IF_FAILED(m_oWheelCounterOutputPin.Transmit(pMediaSample));
    
    RETURN_NOERROR;
}

tResult cArduinoSensors::TransmitVoltData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData)
{    
    //cast to struct
    tVoltageData *arduinoData =  (tVoltageData*) frameData;    
        
    //create new media sample

    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescVoltageData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescVoltageData,pMediaSample,pCoder);

        pCoder->Set("ui32MeasurementCircuit", (tVoid*)&(arduinoData->ui16Measurement));
        pCoder->Set("ui32PowerCircuit", (tVoid*)&(arduinoData->ui16Power));    
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&arduinoTimestamp);
    }    
    
    //transmit media sample over output pin
    pMediaSample->SetTime(inputTimeStamp);
    RETURN_IF_FAILED(m_oVoltOutputPin.Transmit(pMediaSample));
    
    RETURN_NOERROR;
}

tResult cArduinoSensors::TransmitIrData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData)
{
    //cast to struct
    tIrData *arduinoData =  (tIrData*) frameData;
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescIrData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
       
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor    
        __adtf_sample_write_lock_mediadescription(m_pCoderDescIrData,pMediaSample,pCoder);
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&arduinoTimestamp);       
        pCoder->Set("ui16Front_Center_Longrange", (tVoid*)&(arduinoData->ui16FrontCenterLongrange));
        pCoder->Set("ui16Front_Center_Shortrange", (tVoid*)&(arduinoData->ui16FrontCenterShortange));    
        pCoder->Set("ui16Front_Left_Longrange", (tVoid*)&(arduinoData->ui16FrontLeftLongrange));    
        pCoder->Set("ui16Front_Left_Shortrange", (tVoid*)&(arduinoData->ui16FrontLeftShortrange));    
        pCoder->Set("ui16Front_Right_Shortrange", (tVoid*)&(arduinoData->ui16FrontRightShortrange));    
        pCoder->Set("ui16Front_Right_Longrange", (tVoid*)&(arduinoData->ui16FrontRightLongrange));    
        pCoder->Set("ui16Rear_Center_Shortrange", (tVoid*)&(arduinoData->ui16RearCenterShortrange));    
        pCoder->Set("ui16Rear_Left_Shortrange", (tVoid*)&(arduinoData->ui16RearLeftShortrange));    
        pCoder->Set("ui16Rear_Right_Shortrange", (tVoid*)&(arduinoData->ui16RearRightShortrange));
    }    
    
    //transmit media sample over output pin
    pMediaSample->SetTime(inputTimeStamp);
    RETURN_IF_FAILED(m_oIROutputPin.Transmit(pMediaSample));
    
    RETURN_NOERROR;
    
}

tResult cArduinoSensors::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {            
        RETURN_IF_POINTER_NULL(pMediaSample);
        if (pSource == &m_oArduinoCOMInputPin)
        {
            return Process_ArduinoCOMInputPin(pMediaSample);
        }
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged && pSource != NULL)
    {
        cObjectPtr<IMediaType> pType;
        pSource->GetMediaType(&pType);
        if (pType != NULL)
        {
            cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDesc));
            m_pCoderArduinoData = pMediaTypeDesc;                        
        }        
    }    
    RETURN_NOERROR;
}

tResult cArduinoSensors::Process_ArduinoCOMInputPin(IMediaSample *pMediaSample)
{
    if (pMediaSample != NULL && m_pCoderArduinoData != NULL)
    {
        //write values with zero
        tUInt8 SOF = 0;
        tUInt8 ID = 0;
        tUInt32 arduinoTimestamp = 0;
        tUInt8 DataLength = 0;  
        tUInt8 frameData[25];
        
        cMemoryBlock::MemSet(frameData, 0x00, sizeof(frameData));

        {   // focus for sample read lock
            // read-out the incoming Media Sample        
            __adtf_sample_read_lock_mediadescription(m_pCoderArduinoData,pMediaSample,pCoder);
       
            //get values from media sample        
            pCoder->Get("ui8SOF", (tVoid*)&SOF);    
            pCoder->Get("ui8ID", (tVoid*)&ID);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&arduinoTimestamp);
            pCoder->Get("ui8DataLength", (tVoid*)&DataLength);
            pCoder->Get("ui8Data", (tVoid*)&(frameData));  
        }
        
        if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Sensorfilter received: ID %x DataLength %d",ID,DataLength));   

         
        switch(ID)
        {
            case (ID_ARD_SENS_WHEELENC):
                RETURN_IF_FAILED(TransmitCountRPMData(pMediaSample->GetTime(),arduinoTimestamp,frameData));
                break;
            case (ID_ARD_SENS_US):
                RETURN_IF_FAILED(TransmitUsData(pMediaSample->GetTime(),arduinoTimestamp,frameData));
                break;
            case (ID_ARD_SENS_IMU):
                RETURN_IF_FAILED(TransmitAccData(pMediaSample->GetTime(),arduinoTimestamp,frameData));
                RETURN_IF_FAILED(TransmitGyroData(pMediaSample->GetTime(),arduinoTimestamp,frameData));
                break;
            case (ID_ARD_SENS_VOLTAGE):
                RETURN_IF_FAILED(TransmitVoltData(pMediaSample->GetTime(),arduinoTimestamp,frameData));
                break;
            case (ID_ARD_SENS_IR):
                RETURN_IF_FAILED(TransmitIrData(pMediaSample->GetTime(),arduinoTimestamp,frameData));
                break;  
            case (ID_ARD_SENS_STEER_ANGLE):
                RETURN_IF_FAILED(TransmitSteeringAngleData(pMediaSample->GetTime(),arduinoTimestamp,frameData));
                break;    
            default:
                break;            
        }                
    }
    
    RETURN_NOERROR;
}


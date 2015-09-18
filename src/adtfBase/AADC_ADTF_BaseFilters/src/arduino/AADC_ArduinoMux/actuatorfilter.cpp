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

#include "stdafx.h"
#include "actuatorfilter.h"


ADTF_FILTER_PLUGIN("AADC Aktors", OID_ADTF_ACTUATOR_FILTER, cArduinoAktors)

cArduinoAktors::cArduinoAktors(const tChar* __info) : cFilter(__info), m_bDebugModeEnabled(tFalse),m_bDebugModeEnabledWatchdog(tFalse), m_bEmergencyStopBlock(tFalse)
{
    SetPropertyBool("Debug Output to Console",tFalse);
    SetPropertyBool("Omit Watchdog Signal from console output",tTrue);
}

cArduinoAktors::~cArduinoAktors()
{
}

//****************************
tResult cArduinoAktors::CreateInputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid**)&pDescManager, __exception_ptr));
        
	tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBoolSignalInput));

	tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput));

	tChar const * strDescEmergencyStop = pDescManager->GetMediaDescription("tJuryEmergencyStop");
    RETURN_IF_POINTER_NULL(strDescEmergencyStop);
    cObjectPtr<IMediaType> pTypeEmergencyStop = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescEmergencyStop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeEmergencyStop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescEmergencyStop));


    RETURN_IF_FAILED(m_oAccelerateInputPin.Create("accelerate", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oAccelerateInputPin));

    // Steer Angle Input
    RETURN_IF_FAILED(m_oSteerInputPin.Create("steerAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oSteerInputPin));
    
    // Beam Input
    RETURN_IF_FAILED(m_oHeadlightInputPin.Create("headLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oHeadlightInputPin));
    
    // Indicator Input
    RETURN_IF_FAILED(m_oTurnSignalLeftInputPin.Create("turnSignalLeftEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalLeftInputPin));

    RETURN_IF_FAILED(m_oTurnSignalRightInputPin.Create("turnSignalRightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oTurnSignalRightInputPin));

    // hazzard input pin
    RETURN_IF_FAILED(m_oHazardLightsInputPin.Create("hazzardLightsEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oHazardLightsInputPin));

    // Brakelight Input
    RETURN_IF_FAILED(m_oBrakeLightInputPin.Create("brakeLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oBrakeLightInputPin));
    // Reversinglights Input
    RETURN_IF_FAILED(m_oReverseLightInputPin.Create("reverseLightsEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oReverseLightInputPin));
        
    // Gear Input
       RETURN_IF_FAILED(m_oWatchdogInputPin.Create("Watchdog_Alive_Flag", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oWatchdogInputPin));
  
    // NotAus Output
    RETURN_IF_FAILED(m_oEmergencyStopInputPin.Create("Emergency_Stop", pTypeEmergencyStop, this));
    RETURN_IF_FAILED(RegisterPin(&m_oEmergencyStopInputPin));
       
    
    RETURN_NOERROR;
}

//*****************************************************************************

tResult cArduinoAktors::CreateOutputPins(__exception)
{    
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescArduino = pDescManager->GetMediaDescription("tArduinoData");
    RETURN_IF_POINTER_NULL(strDescArduino);
    cObjectPtr<IMediaType> pTypeArduinoData = new cMediaType(0, 0, 0, "tArduinoData", strDescArduino,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(m_oArduinoCOMOutPin.Create("ArduinoCOM_output", pTypeArduinoData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oArduinoCOMOutPin));
    RETURN_IF_FAILED(pTypeArduinoData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescArduinoData)); 
    
    RETURN_NOERROR;    
}

tResult cArduinoAktors::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        tResult nResult = CreateOutputPins();
        if (IS_FAILED(nResult))
        {
            THROW_ERROR_DESC(nResult, "Failed to create Output Pins");
        }
        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult))
        {
            THROW_ERROR_DESC(nResult, "Failed to create Input Pins");
        }
    }
    else if (eStage == StageNormal)
    {
        m_bDebugModeEnabledWatchdog = GetPropertyBool("Omit Watchdog Signal from console output");
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    RETURN_NOERROR;
}

tResult cArduinoAktors::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cArduinoAktors::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cArduinoAktors::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}


tResult cArduinoAktors::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
         
        if (pSource == &m_oHeadlightInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_DATA_HEAD);
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_DATA_BACK);
        }        
        
        if (pSource == &m_oTurnSignalLeftInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_DATA_TURNLEFT);
        }

        if (pSource == &m_oTurnSignalRightInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_DATA_TURNRIGHT);    
        }

        if (pSource == &m_oHazardLightsInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_DATA_HAZZARD);    
        }

        if (pSource == &m_oBrakeLightInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_DATA_BRAKE);
        }
        
        if (pSource == &m_oReverseLightInputPin)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_DATA_REVERSE);
        }
            
        if (pSource == &m_oEmergencyStopInputPin)
        {
            ProcessEmergencyStop(pMediaSample);
        }

        if (pSource == &m_oSteerInputPin)
        {
            ProcessActuatorValue(pMediaSample, ID_ARD_ACT_STEER_ANGLE);
        }
        
        if (pSource == &m_oAccelerateInputPin)
        {
            ProcessActuatorValue(pMediaSample, ID_ARD_ACT_ACCEL_SERVO);
        }
        
        if (pSource == &m_oWatchdogInputPin)
        {
            ProcessWatchdog(pMediaSample);
        }

    }
    RETURN_NOERROR;
}

tResult cArduinoAktors::ProcessEmergencyStop(IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    if (pMediaSample != NULL && m_pCoderDescEmergencyStop != NULL)
    {
        tBool boolValue = tFalse;
        
        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescEmergencyStop,pMediaSample,pCoder);      
        
            pCoder->Get("bEmergencyStop", (tVoid*)&boolValue);     
        }
        
                
          
        //set watchdog active in arduino to tFalse
        if(boolValue == tTrue)
        {
			m_bEmergencyStopBlock = tTrue;
            tUInt8 ui8Data = 0x00;
            BuildArduinoFrame(ID_ARD_ACT_MOT_RELAIS,1,&ui8Data);        //Set motorrelais to 0
            BuildArduinoFrame(ID_ARD_ACT_WD_ENABLE,1,&ui8Data);     //set watchdog active in arduino to tFalse    
            LOG_WARNING(cString::Format("Emergency_Stop now!")); 
        }
    }
    RETURN_NOERROR;
}


tResult cArduinoAktors::ProcessWatchdog(IMediaSample* pMediaSample)
{        
     __synchronized_obj(m_oWatchDogCritSection);
    if (pMediaSample != NULL && m_pCoderDescBoolSignalInput != NULL && m_bEmergencyStopBlock == tFalse)
    {
        //write values with zero
        tBool bValue = tFalse;
        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescBoolSignalInput,pMediaSample,pCoder);

            //get values from media sample        
            pCoder->Get("bValue", (tVoid*)&bValue); 
        }
                  
                    
        tUInt8 ui8Data = static_cast<tUInt8> (bValue);
        if (bValue == tTrue)
        {
            BuildArduinoFrame(ID_ARD_ACT_WD_TOGGLE,1,&ui8Data);
        }

    }
    RETURN_NOERROR;
}
    

tResult cArduinoAktors::ProcessActuatorValue( IMediaSample* pMediaSample, tUInt8 chID )
{  
    __synchronized_obj(m_oActuatorCritSection);
    if (pMediaSample != NULL && m_pCoderDescSignalInput != NULL)
    {
        //write values with zero
        tFloat32 f32Value = 0;
        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescSignalInput,pMediaSample,pCoder);  
            //get values from media sample        
            pCoder->Get("f32Value", (tVoid*)&f32Value);    
        }
                
        // because the ARDUINO protocol defines the range between 0...180
        // the values must be limited 
        if (f32Value > 180.0f)
        {
            f32Value = 180.0f;
            LOG_WARNING("Actuator value out of range. Setting to max. 180");
        }
        if (f32Value < 0.0f)
        {
            f32Value = 0.0f;
            LOG_WARNING("Actuator value out of range. Setting to min. 0");
        }
        // cast from float to tUInt8 with correct rounding
        tUInt8 ui8Data = static_cast<tUInt8> (f32Value + 0.5f);
        BuildArduinoFrame(chID,1,&ui8Data);
    }
    RETURN_NOERROR;
}

tResult cArduinoAktors::ProcessLights(IMediaSample* pMediaSample, tUInt8 ui8LightID)
{   
    __synchronized_obj(m_oLightsCritSection); 
    if (pMediaSample != NULL && m_pCoderDescBoolSignalInput != NULL)
    {
        tBool bValue = tFalse;
        tUInt8 achData[2];

        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescBoolSignalInput,pMediaSample,pCoder);
        
            //get values from media sample        
            pCoder->Get("bValue", (tVoid*)&bValue);    
        }
        
        achData[0] = ui8LightID;
        bValue == tTrue ? achData[1] = 1: achData[1] = 0;

        if (m_bDebugModeEnabled) 
        {
            LOG_INFO(cString::Format("Actuatorfilter sent Light: ID %x Value %d",achData[0], achData[1]));
        }

        BuildArduinoFrame(ID_ARD_ACT_LIGHT_DATA,2,achData);
    }
    RETURN_NOERROR;
 
}

tResult cArduinoAktors::BuildArduinoFrame( tUInt8 ch8Id, tUInt8 i8DataLength, tUInt8 *pchData )
{    
    __synchronized_obj(m_oTransmitCritSection); 
    // build empty frame
    tUInt8 achFrameData[25];
    cMemoryBlock::MemSet(achFrameData, 0x00, sizeof(achFrameData));
    cMemoryBlock::MemCopy(achFrameData, pchData, i8DataLength);
            
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescArduinoData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescArduinoData,pMediaSample,pCoder);

        tUInt8 i8SOF = ID_ARD_SOF; 
        tUInt32 ui32ArduinoTimestamp = 0;  

        pCoder->Set("ui8SOF", (tVoid*)&i8SOF);    
        pCoder->Set("ui8ID", (tVoid*)&ch8Id);
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&(ui32ArduinoTimestamp));

        pCoder->Set("ui8DataLength", (tVoid*)&i8DataLength);
        pCoder->Set("ui8Data", (tVoid*)&(achFrameData));
    }     

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oArduinoCOMOutPin.Transmit(pMediaSample));       
    // check if output is enabled and id is not watchdog   OR         id is watchdog and watchdog output is enabled and output is enabled
    if (i8DataLength > 0)
    {
        if ((m_bDebugModeEnabled && ch8Id != ID_ARD_ACT_WD_TOGGLE) || (ch8Id == ID_ARD_ACT_WD_TOGGLE && !(m_bDebugModeEnabledWatchdog) && m_bDebugModeEnabled) ) LOG_INFO(cString::Format("Actuatorfilter sent: ID %x DataLength %d First Value %d",ch8Id,i8DataLength,achFrameData[0]));   
    }
    RETURN_NOERROR;
}





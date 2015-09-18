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
#include "ControllerFilter.h"

ADTF_FILTER_PLUGIN("AADC PID Controller", OID_ADTF_PIDCONTROLLER, ControllerFilter)

ControllerFilter::ControllerFilter(const tChar* __info) : cFilter(__info), m_lastMeasuredError(0), m_setPoint(0), m_lastSampleTime(0)
{
    SetPropertyFloat("Controller Kp value",1);
    SetPropertyFloat("Controller Ki value",1);
    SetPropertyFloat("Controller Kd value",1);

    SetPropertyInt("Sample Interval [msec]",1);
    SetPropertyBool("use automatically calculated sample interval",1);
    SetPropertyInt("Controller Typ", 1);
    SetPropertyStr("Controller Typ" NSSUBPROP_VALUELISTNOEDIT, "1@P|2@PI|3@PID");
}

ControllerFilter::~ControllerFilter()
{
}

tResult ControllerFilter::CreateInputPins(__exception)
{    

    
    
    
    RETURN_IF_FAILED(m_oInputMeasured.Create("measured variable", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputMeasured));
    RETURN_IF_FAILED(m_oInputSetPoint.Create("set point", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSetPoint));
    RETURN_NOERROR;
}

tResult ControllerFilter::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
    
    
    RETURN_IF_FAILED(m_oOutputManipulated.Create("manipulated variable", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputManipulated));
    RETURN_NOERROR;
}

tResult ControllerFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
        {
                CreateInputPins(__exception_ptr);
                CreateOutputPins(__exception_ptr);
        }
    else if (eStage == StageNormal)
        {

        }
    else if(eStage == StageGraphReady)
        {    
            
        }
                
    RETURN_NOERROR;
}

tResult ControllerFilter::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult ControllerFilter::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult ControllerFilter::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult ControllerFilter::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL) 
    {

        RETURN_IF_POINTER_NULL( pMediaSample);
        
        if (pSource == &m_oInputMeasured)
        {
                cObjectPtr<IMediaCoder> pCoder;
                RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

                //write values with zero
                tFloat32 value = 0;
                tUInt32 timeStamp = 0;
                tFloat32 outputData = 0;
                
                //get values from media sample        
                pCoder->Get("f32Value", (tVoid*)&value);
                pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                m_pCoderDescSignal->Unlock(pCoder);   
                   
                //calculation
                m_measuredVariable = value;
                outputData =getControllerValue(value);
                
                //create new media sample
                cObjectPtr<IMediaSample> pMediaSample;
                AllocMediaSample((tVoid**)&pMediaSample);

                //allocate memory with the size given by the descriptor
                cObjectPtr<IMediaSerializer> pSerializer;
                m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
                tInt nSize = pSerializer->GetDeserializedSize();
                pMediaSample->AllocBuffer(nSize);

                //write date to the media sample with the coder of the descriptor
                m_pCoderDescSignal->WriteLock(pMediaSample, &pCoder);    

                pCoder->Set("f32Value", (tVoid*)&(outputData));    
                pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
                m_pCoderDescSignal->Unlock(pCoder);

                //transmit media sample over output pin
                RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
                RETURN_IF_FAILED(m_oOutputManipulated.Transmit(pMediaSample));            
                
        }
        else if (pSource == &m_oInputSetPoint)
        {                
            cObjectPtr<IMediaCoder> pCoder;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;
            
            //get values from media sample        
            pCoder->Get("f32Value", (tVoid*)&value);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoder); 
            m_setPoint = value;
        }
        else
            RETURN_NOERROR;
        

    }
    RETURN_NOERROR;
}


tFloat32 ControllerFilter::getControllerValue(tFloat32 measuredValue)
{
    //calculate the sample time in milliseceonds if necessary
    tInt sampleTime;
    if (GetPropertyBool("use automatically calculated sample interval",1)==tTrue)
        sampleTime = tInt(GetTime() - m_lastSampleTime)*1000;
    else 
        sampleTime    = GetPropertyInt("Sample Intervall [msec]",1);
    m_lastSampleTime = GetTime();
    
    //the three controller algorithms
    if (GetPropertyInt("Controller Typ", 1)==1)
        {
        //P-Regler y = Kp * e
        tFloat32 result = tFloat32(GetPropertyFloat("Controller Kp value",1)*(m_setPoint-measuredValue));
        return result;
        }
    else if(GetPropertyInt("Controller Typ", 1)==2) //PI- Regler
        {
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum
        m_accumulatedVariable +=(m_setPoint-measuredValue);
        tFloat32 result = tFloat32(GetPropertyFloat("Controller Kp value",1)*(m_setPoint-measuredValue) \
            +GetPropertyFloat("Controller Ki value",1)*sampleTime*m_accumulatedVariable);        
        return result;
        }
    else if(GetPropertyInt("Controller Typ", 1)==3)
        {
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum + Kd * (e – ealt)/Ta
        //ealt = e
        m_accumulatedVariable +=(m_setPoint-measuredValue);
        tFloat32 result =  tFloat32(GetPropertyFloat("Controller Kp value",1)*(m_setPoint-measuredValue) \
            +tFloat32(GetPropertyFloat("Controller Ki value",1))*sampleTime*m_accumulatedVariable) \
            +tFloat32(GetPropertyFloat("Controller Kd value",1))*((m_setPoint-measuredValue)-m_lastMeasuredError)/sampleTime;    
        m_lastMeasuredError = (m_setPoint-measuredValue);
        return result;
        }    
    else
        return 0;
}

tTimeStamp ControllerFilter::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

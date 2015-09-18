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
#include "cSlidingWindow.h"
#include "RPMFilter.h"

ADTF_FILTER_PLUGIN("AADC RPM Calculation", OID_ADTF_RPM_CALCULATION, RPMCalculation)

RPMCalculation::RPMCalculation(const tChar* __info) : cFilter(__info), m_SlidingWindowCntLeftWheel(10),m_SlidingWindowCntRightWheel(10)
{
}

RPMCalculation::~RPMCalculation()
{
}

tResult RPMCalculation::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
        {
            cObjectPtr<IMediaDescriptionManager> pDescManager;
            RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

            tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
            RETURN_IF_POINTER_NULL(strDescSignalValue);        
            cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
    
            
            RETURN_IF_FAILED(m_oInputWheelCounterLeft.Create("wheel_speed_sensor_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputWheelCounterLeft));
            RETURN_IF_FAILED(m_oInputWheelCounterRight.Create("wheel_speed_sensor_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oInputWheelCounterRight));

            RETURN_IF_FAILED(m_oOutputRPMLeft.Create("RPM_left_Wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputRPMLeft));
            RETURN_IF_FAILED(m_oOutputRPMRight.Create("RPM_right_Wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputRPMRight));

    RETURN_NOERROR;
        }
    else if (eStage == StageNormal)
        {
            
        }
    else if(eStage == StageGraphReady)
        {    
            
        }
                
    RETURN_NOERROR;
}

tResult RPMCalculation::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {        
        if (pSource == &m_oInputWheelCounterLeft)
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

                //processing the data
                m_SlidingWindowCntLeftWheel.addNewValue(value,GetTime());
                tFloat32 outputData = 0;
                if (m_SlidingWindowCntLeftWheel.getEndValue()!=0)                            
                    outputData = calcLatestRpm(m_SlidingWindowCntLeftWheel.getEndValue(),m_SlidingWindowCntLeftWheel.getBeginValue(), 
                        m_SlidingWindowCntLeftWheel.getEndTime(),m_SlidingWindowCntLeftWheel.getBeginTime());
                else
                    outputData=0;
        
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

                    pCoderOutput->Set("f32Value", (tVoid*)&(outputData));    
                    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    


                }
                //transmit media sample over output pin
                pNewMediaSample->SetTime(pMediaSample->GetTime());
                m_oOutputRPMLeft.Transmit(pNewMediaSample);
            }
        else if (pSource == &m_oInputWheelCounterRight)
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
                //processing the data
                m_SlidingWindowCntRightWheel.addNewValue(value,GetTime());
                tFloat32 outputData = 0;
                if (m_SlidingWindowCntRightWheel.getEndValue()!=0)                            
                    outputData = calcLatestRpm(m_SlidingWindowCntRightWheel.getEndValue(),m_SlidingWindowCntRightWheel.getBeginValue(), 
                        m_SlidingWindowCntRightWheel.getEndTime(),m_SlidingWindowCntRightWheel.getBeginTime());
                else
                    outputData=0;
        
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

                    pCoderOutput->Set("f32Value", (tVoid*)&(outputData));    
                    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                }    

                //transmit media sample over output pin
                pNewMediaSample->SetTime(pMediaSample->GetTime());
                m_oOutputRPMRight.Transmit(pNewMediaSample);
            }
    }
    RETURN_NOERROR;
}

tTimeStamp RPMCalculation::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tFloat32 RPMCalculation::calcLatestRpm( tFloat32 counterValue, tFloat32 lastCounterValue,tTimeStamp currentTimestamp, tTimeStamp lastTimestamp )
{       
  long deltaTime = long(currentTimestamp - lastTimestamp);            //microseconds
  unsigned long deltaCounter = long(counterValue) - long(lastCounterValue);  
  tFloat32 deltaRounds = tFloat32(deltaCounter)/8.0f;  //one round = 8 counts
  tFloat32 rpm;
  if (deltaTime!=0)
            {
            rpm = tFloat32(deltaRounds/deltaTime * 1.0E6*60.0);        //rounds/time(usec) * usec/60 sec
            //LOG_INFO(adtf_util::cString::Format("counter %d...%d , rpm = %f ",counterValue,lastCounterValue,rpm));    
            }
  else {
        rpm = 0;
        //LOG_INFO("rpm = NULL ");    
        }
  
  return rpm;

}

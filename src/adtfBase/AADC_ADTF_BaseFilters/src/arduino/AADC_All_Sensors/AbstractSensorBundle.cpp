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

cAbstractSensorsBundle::cAbstractSensorsBundle(const tChar* __info):cFilter(__info){

    SetPropertyBool("Samplerate Output To Console", tFalse);
    SetPropertyInt("Length List For Samplerate Measurement", 500);
    
};

tResult cAbstractSensorsBundle::Init(tInitStage eStage, __exception){
        RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

        if (eStage == StageFirst)
        {
                CreateInputPins();
                CreateOutputPins();
                m_sampleOutputEnable = GetPropertyBool("Samplerate Output To Console");
                m_sampleListLength = GetPropertyInt("Length List For Samplerate Measurement");
                m_lastSampleTime =0;
        }
        
            
        RETURN_NOERROR;
};

tResult cAbstractSensorsBundle::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {

        RETURN_IF_POINTER_NULL( pMediaSample);
        
        if (m_sampleOutputEnable) ProcessSamplingRate(pMediaSample->GetTime());
        
        ProcessData(pMediaSample);
               
      }
     else if (nEventCode == IPinEventSink::PE_MediaTypeChanged && pSource != NULL)
    {
        cObjectPtr<IMediaType> pType;
        pSource->GetMediaType(&pType);
        if (pType != NULL)
        {
            cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDesc));
            m_pCoderDescInput = pMediaTypeDesc;
                        
        }
        
    }

    RETURN_NOERROR;
}

tResult cAbstractSensorsBundle::ProcessSamplingRate(const tTimeStamp t)
{
    if (m_lastSampleTime == 0)    
        {
        m_lastSampleTime=t;
        }
    else
        {
        if (m_SampleTimes.size() <= m_sampleListLength)
            {
            m_SampleTimes.push_back(t-m_lastSampleTime);
            m_accSampleTime += (t-m_lastSampleTime);
            m_lastSampleTime=t;
            }
        if (m_SampleTimes.size() == m_sampleListLength)
            {
            tFloat64 median=(tFloat64) (1/((tFloat64)(m_accSampleTime)/((tFloat64)(m_sampleListLength)*1000000.0)));
            tFloat64 variance=0;
            for (tUInt i =0;i<m_SampleTimes.size();i++)
                {
                variance += pow((1/((tFloat64)(m_SampleTimes.front())/1000000.0)-median),2);
                m_SampleTimes.pop_front();
                }
            variance = variance/(m_sampleListLength-1);
            PrintSamplingRate(median, variance);
              
            m_SampleTimes.clear();
            m_accSampleTime = 0;
            }
           }
    
    RETURN_NOERROR;
}



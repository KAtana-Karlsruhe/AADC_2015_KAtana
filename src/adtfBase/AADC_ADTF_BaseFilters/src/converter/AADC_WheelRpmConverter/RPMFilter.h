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
* $Author:: spiesra $  $Date:: 2014-09-16 13:29:48#$ $Rev:: 26104   $
**********************************************************************/

#ifndef _RPMFILTER_H_
#define _RPMFILTER_H_

#define OID_ADTF_RPM_CALCULATION "adtf.aadc.rpmcalculation"

/*!
* This is filter calculates the rounds per second of the wheels.
*/
class RPMCalculation : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_RPM_CALCULATION, "AADC RPM Calculation", OBJCAT_DataFilter, "RPM Calculation Filter", 1, 0, 0, "Beta Version");

        /*!input pin for the the counter of the left wheel */
        cInputPin m_oInputWheelCounterLeft;
        /*!input pin for the the counter of the right wheel */
        cInputPin m_oInputWheelCounterRight;
        /*!output pin for the the rpm of the left wheel */
        cOutputPin m_oOutputRPMLeft;
        /*!output pin for the the rpm of the right wheel */
        cOutputPin m_oOutputRPMRight;

    public:
        RPMCalculation(const tChar* __info);
        virtual ~RPMCalculation();
    
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);        
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    
    private:
        /*!gets the actual time as a tTimeStamp */
        tTimeStamp GetTime();
        
        /*!calculates the RPM of the wheel with the given parameters
        @param counterValue        the actual value of the counter
        @param lastCounterValue    the last value of the counter which must be a certain interval ago (defined with the size of the sliding window)
        @param currentTimestamp    the actual timestamp
        @param lastTimestamp    the timestamp of the lastCounterValue
        */
        tFloat32 calcLatestRpm( tFloat32 counterValue, tFloat32 lastCounterValue,tTimeStamp currentTimestamp, tTimeStamp lastTimestamp );
        
        /*!sliding window filter for the left wheel*/
        cSlidingWindow m_SlidingWindowCntLeftWheel;
        /*!sliding window filter for the right wheel*/
        cSlidingWindow m_SlidingWindowCntRightWheel;
        
        /*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;    
};



//*************************************************************************************************

#endif // _RPMFILTER_H_


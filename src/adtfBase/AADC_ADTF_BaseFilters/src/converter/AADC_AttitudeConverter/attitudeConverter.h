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


#ifndef _QUATERNION_FILTER_H_
#define _QUATERNION_FILTER_H_

#define OID_ADTF_QUATERNION_FILTER "adtf.aadc.quaternion"

using namespace adtf;
/*!
* This is filter converts the attitude from the quaternions system to the euler angles
*/
class cAttitudeFilter : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_QUATERNION_FILTER, "AADC Attitude Converter", OBJCAT_DataFilter, "Attitude Converter", 1, 0, 0, "Beta Version");    
    
    public:
        cAttitudeFilter(const tChar* __info);
        virtual ~cAttitudeFilter();
    
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    
    private:
        /*! creates all the output pins */
        tResult CreateOutputPins(__exception = NULL);
        /*! creates all the input pins */
        tResult CreateInputPins(__exception = NULL);
        
        /*! calculates the euler angles from the quaternion
        @param yaw_out pointer to the yaw angle
        @param pitch_out pointer to the pitch angle
        @param roll_out pointer ot the roll angle
        */
        tResult calulateEulerAngles(tFloat32 &yaw_out, tFloat32 &pitch_out, tFloat32 &roll_out); 
        /*! this function transmits to calculated angle
        @param sampleTimeStamp timestamp of the sample
        @param timeStampValue
        @param yaw yaw value
        @param pitch pitch value
        @param roll roll value
        */
        tResult TransmitAngles(tTimeStamp sampleTimeStamp, const tTimeStamp timeStampValue, const tFloat32 yaw, const tFloat32 pitch, const tFloat32 roll);
        
        /*! quaternion w - component */
        tFloat32 qW;
        /*! quaternion x - component */
        tFloat32 qX;
        /*! quaternion y - component */
        tFloat32 qY;
        /*! quaternion z - component */
        tFloat32 qZ;
        /*! input pin for quaternion w - component */
        cInputPin m_oInput_qW;
        /*! input pin for  quaternion x - component */
        cInputPin m_oInput_qX;
        /*! input pin for quaternion y - component */
        cInputPin m_oInput_qY;
        /*! input pin for quaternion z - component */
        cInputPin m_oInput_qZ;
        /*! output pin for pitch angle */
        cOutputPin m_oOutputPitch;
        /*! output pin for yaw angle */
        cOutputPin m_oOutputYaw;
        /*! output pin for roll angle */
        cOutputPin m_oOutputRoll;
        /*! output mode */
        tInt m_EulerAnglesType; 
        /*!.wheter prints has to made to the console */
        tBool m_bDebugModeEnabled;
        
        /*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
};



//*************************************************************************************************

#endif // _QUATERNION_FILTER_H_

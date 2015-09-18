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

/*! \brief Calibration Scaling
 *         
 *  Mit diesem Filter können Sensor- oder Aktorwerte linear skaliert werden. Dabei wird der Eingangswert  mit dem in den Eigenschaften gegebenen Wert multipliziert. 
 */

#ifndef _SENSORCALIBRATION_H_
#define _SENSORCALIBRATION_H_


#define OID_ADTF_SENSOR_SCALING "adtf.aadc.calibrationScaling"


/*!
This filter can be used to scale data with a simple scale factor
*/
class SensorScaling : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SENSOR_SCALING, "AADC Calibration Scaling", OBJCAT_DataFilter, "Sensor Value Scaling Filter", 1, 0, 1, "Beta Version");    

        cInputPin m_oInput;                //input pin for the raw value
        cOutputPin m_oOutput;            //output pin for the calibrated value

    public:
        SensorScaling(const tChar* __info);
        virtual ~SensorScaling();
    
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);        
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    
    private:
        /*! creates all the input Pins*/
        tResult CreateInputPins(__exception = NULL);
        /*! creates all the output Pins*/
        tResult CreateOutputPins(__exception = NULL);
        /*! factor which used for the method 1 of simple scaling*/
        tFloat32 m_ScaleFactor;
        
        /*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;            
        
};



//*************************************************************************************************

#endif // _SENSORCALIBRATION_H_


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

#ifndef _GYROUSSBUNDLE_FILTER_H_
#define _GYROUSSBUNDLE_FILTER_H_

#include "AbstractSensorBundle.h"

#define OID_ADTF_USSBUNDLE_FILTER "adtf.aadc.ultrasonicSignals"
/*!
* This is filter divides the ultra sonic sensors struct in the different components. The values have to be calibrated afterwards.
*/
class cUSSSensorBundle : public cAbstractSensorsBundle{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_USSBUNDLE_FILTER, "AADC Ultrasonic Signals", OBJCAT_SensorDevice , "Ultrasonic Signals ", 1, 0,0, "Beta Version");    
public:
    cUSSSensorBundle(const tChar* __info);
    virtual ~cUSSSensorBundle(){};
protected:
    /*! creates all the output pins 
    */
    tResult CreateOutputPins(__exception = NULL);
    /*! creates all the input pins 
    */
    tResult CreateInputPins(__exception = NULL);
    /*! processes the incoming mediasample
    @param pMediaSample the incoming mediasample
    */
    tResult ProcessData(IMediaSample* pMediaSample);
    /*! transmits the new media sample
    @param inputTimeStamp timestamp of the incoming mediasample
    @param front_left value of front left sensor
    @param front_right value of front right sensor
    @param rear_left value of rear left sensor
    @param rear_right value of rear left sensor
    @param timestampValue the timestamp from the arduino of the incoming sample
    */
    tResult TransmitData(tTimeStamp inputTimeStamp, tFloat32 front_left,tFloat32 front_right, tFloat32 rear_left, tFloat32 rear_right, tUInt32 timestampValue);
    /* prints the results of the sampling rate measurement to the console
    @param median the median value which is printed to console
    @param variance the variance value which is printed to console
    */
    tResult PrintSamplingRate(tFloat64 median, tFloat64 variance);
private:

    cOutputPin m_outputPin_front_left;    /**< output pin for uss front left data */
    cOutputPin m_outputPin_front_right;    /**< output pin for uss front right data */
    cOutputPin m_outputPin_rear_left;    /**< output pin for uss rear left data */
    cOutputPin m_outputPin_rear_right;    /**< output pin for uss rear right data */

    cInputPin m_oInput;                    /**< input pin for incoming struct */
};

#endif

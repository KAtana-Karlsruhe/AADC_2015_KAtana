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



#ifndef _GYROSENSORBUNDLE_FILTER_H_
#define _GYROSENSORBUNDLE_FILTER_H_


#define OID_ADTF_GYROBUNDLE_FILTER "adtf.aadc.gyroscopeSignals"

class cGyroSensorBundle : public cAbstractSensorsBundle
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_GYROBUNDLE_FILTER, "AADC Gyroscope Signals", OBJCAT_SensorDevice, "Gyroscope Signals", 1, 0,0, "Beta Version");    
public:
    cGyroSensorBundle(const tChar* __info);
    virtual ~cGyroSensorBundle(){};
protected:
    /*! creates all the output pins 
    */
    tResult CreateOutputPins(__exception = NULL);
    /*! creates all the tInt pins 
    */
    tResult CreateInputPins(__exception = NULL);
    /*! processes the incoming mediasample
    @param pMediaSample the incoming mediasample
    */
    tResult ProcessData(IMediaSample* pMediaSample);
    /*! transmits the new media sample
    @param inputTimeStamp timestamp of the incoming mediasample
    @param gyro_w w component of the gyroscope data
    @param gyro_x x component of the gyroscope data
    @param gyro_y y component of the gyroscope data
    @param gyro_z z component of the gyroscope data
    @param timestampValue the timestamp from the arduino of the incoming sample
    */
    tResult TransmitData(tTimeStamp inputTimeStamp, tFloat32 gyro_w, tFloat32 gyro_x,tFloat32 gyro_y, tFloat32 gyro_z,tUInt32 timestampValue);
    /* prints the results of the sampling rate measurement to the console
    @param median the median value which is printed to console
    @param variance the variance value which is printed to console
    */
    tResult PrintSamplingRate(tFloat64 median, tFloat64 variance);
private:
    cOutputPin m_outputPin_gyro_w;        /**< output pin for the w component of the gyroscope data */
    cOutputPin m_outputPin_gyro_x;        /**< output pin for the x component of the gyroscope data */
    cOutputPin m_outputPin_gyro_y;        /**< output pin for the y component of the gyroscope data */
    cOutputPin m_outputPin_gyro_z;        /**< output pin for the z component of the gyroscope data */

    cInputPin m_oInput;                    /**< input pin for incoming struct */
};

#endif

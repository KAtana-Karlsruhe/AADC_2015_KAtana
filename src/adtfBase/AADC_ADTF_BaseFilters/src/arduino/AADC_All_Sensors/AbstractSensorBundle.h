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

#ifndef _ABSTRACTSENSORBUNDLE_FILTER_H_
#define _ABSTRACTSENSORBUNDLE_FILTER_H_

#define OID_ADTF_ABSTRACTBUNDLE_FILTER "adtf.aadc.sensorsBundle.cAbstractSensorsBundle"
/*!
* This is an abstract interface for all the sensors
*/
class cAbstractSensorsBundle : public adtf::cFilter
{    
public:
    cAbstractSensorsBundle(const tChar* __info);
    virtual ~cAbstractSensorsBundle(){};
protected:    
    /*!needs to be implemented for derivied classes    */
    virtual tResult CreateOutputPins(__exception = NULL) = 0;
    /*!needs to be implemented for derivied classes     */
    virtual tResult CreateInputPins(__exception = NULL) = 0;
    /*! this functions calculates the Sampling Rate of the incoming data packages and prints the median and the variance of the rate to the console if it is enabled in the filter properties
    @param t
    */
    tResult ProcessSamplingRate(const tTimeStamp t);
    /*! this functions prints the sampling rate measurement results to the console. it has to be implemented by each filter on its own because different strings are printed to the console
    @param median   the median of the sampling rate
    @param variance the variance of the sampling rate
    */
    virtual tResult PrintSamplingRate(tFloat64 median, tFloat64 variance) = 0;
    /*! this functions processes the incoming pMediaSample. It has to be implemented by each filter on its own
    @param pMediaSample the incoming media sample    
    */
    virtual tResult ProcessData(IMediaSample* pMediaSample) = 0;
    

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*!a list for the timestamps to calculate the sampling rate*/
    list<tTimeStamp> m_SampleTimes;
    /*! the last timestamp of the media sample to calculate the difference*/
    tTimeStamp m_lastSampleTime;
    /*! the accumulated timestamps to calculate the median of the sampling rate*/
    tTimeStamp m_accSampleTime;    
    /*! the length of the list, i.e. the number of samples over which the rate is calculated*/
    tUInt m_sampleListLength;
    /*! holds wheter the sampling rate is calculated or not*/
    tBool m_sampleOutputEnable;
    
    /*! Coder Descriptor for the input pin*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInput;
    /*! Coder Descriptor for the output pin*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutput;
};

#endif

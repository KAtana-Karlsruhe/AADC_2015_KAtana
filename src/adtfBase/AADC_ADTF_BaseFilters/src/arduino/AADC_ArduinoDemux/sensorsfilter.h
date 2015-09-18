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

#ifndef _SENSOR_FILTER_H_
#define _SENSOR_FILTER_H_

#include "stdafx.h"
#define OID_ADTF_ARDUINO_FILTER "adtf.aadc.sensors"

/*!
 *  This filter receives the data from the Arduino Communication Filter and divides them in the different sensors
*/
class cArduinoSensors : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ARDUINO_FILTER, "AADC Arduino Sensors", OBJCAT_SensorDevice, "Arduino Sensors", 1, 0,0, "Beta Version");    
    
    protected:
        cInputPin                m_oArduinoCOMInputPin;              /*!< input pin for data from arduino com*/
        cOutputPin                m_oUSSOutputPin;                    /*!< Output Pin for USS*/
        cOutputPin                m_oSteeringAngleOutputPin;            /*!< Output Pin for steering_angle*/
        cOutputPin                m_oACCOutputPin;                    /*!< Output Pin for ACC*/
        cOutputPin                m_oGyroOutputPin;                    /*!< Output Pin for GYRO*/
        cOutputPin                m_oWheelCounterOutputPin;            /*!< Output Pin for WHEEL COUNTER*/
        cOutputPin                m_oVoltOutputPin;                    /*!< Output Pin for Volt Data    */    
        cOutputPin                m_oIROutputPin;                        /*!< Output Pin for IR Data*/    
    
        tBool m_bDebugModeEnabled;                  /*!< Output Pin for Photo Data  */    

    public:
        cArduinoSensors(const tChar* __info);
        virtual ~cArduinoSensors();

    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    protected:
        /*! transmits and splits the steering_angle package to the output
        @param inputTimeStamp the timestamp of the incoming sample        
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        */
        tResult TransmitSteeringAngleData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData);
        
        /*! transmits and splits the gyro package to the output
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        */
        tResult TransmitGyroData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData);
        
        /*! transmits and splits the accelerometer package to the output
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        */
        tResult TransmitAccData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData);
        
        /*! transmits and splits the ultrasonic package to the output
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        */
        tResult TransmitUsData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData);
        
        /*! transmits and splits the wheel encoder package to the output
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        */
        tResult TransmitCountRPMData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData);
        
        /*! transmits and splits the voltage measurement package to the output
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        */
        tResult TransmitVoltData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData);
        
        /*! transmits and splits the infrared package to the output
        @param inputTimeStamp the timestamp of the incoming sample
        @param arduinoTimestamp the timestamp from the arduino
        @param frameData pointer to the data in the frame
        */
        tResult TransmitIrData(tTimeStamp inputTimeStamp, const tUInt32 arduinoTimestamp,const tUInt8* frameData);
        
        /*! resolves the incoming mediasample and filters to the different transmit functions            
        @param pMediaSample incoming mediasample
        */
        tResult Process_ArduinoCOMInputPin(IMediaSample *pMediaSample);
        
        
        /*! Coder Descriptors */
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSteeringAngleData; /*!< descriptor for steeringAngleData*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescVoltageData; /*!< descriptor for voltageData*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescWheelEncoderData; /*!< descriptor for wheelEncoderData*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescIrData; /*!< descriptor for irData*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescUsData; /*!< descriptor for usData*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescAccData; /*!< descriptor for accData*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescGyroData; /*!< descriptor for gyroData*/
        cObjectPtr<IMediaTypeDescription> m_pCoderArduinoData; /*!< descriptor for input data from Arduino*/
        
    private:
        inline tResult CreateOutputPins(__exception = NULL);
        inline tResult CreateInputPins();
};

//*************************************************************************************************

#endif // _ARDUINO_FILTER_H_


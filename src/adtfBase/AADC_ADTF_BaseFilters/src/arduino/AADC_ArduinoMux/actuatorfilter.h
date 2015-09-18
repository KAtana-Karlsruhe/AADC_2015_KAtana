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


#ifndef _AKTORS_FILTER_H_
#define _AKTORS_FILTER_H_


#define OID_ADTF_ACTUATOR_FILTER "adtf.aadc.actuators"
//! class of filter for the processing of actuator values
    /*!
    This filter processes of actuator values
    */
class cArduinoAktors : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ACTUATOR_FILTER, "AADC Arduino Actuator", OBJCAT_SensorDevice, "Actuator Filter", 1, 2, 0,"");
    protected:
        cOutputPin                m_oArduinoCOMOutPin;                /*!< output pin to arduino  */
        cInputPin                m_oAccelerateInputPin;              /*!< Input Pin for accelerate values; values must be between 0...180*/
        cInputPin                m_oReverseLightInputPin;            /*!< Input Pin for reverse light messages */
        cInputPin                m_oHeadlightInputPin;                /*!< Input Pin for beam messages  */
        cInputPin                m_oTurnSignalLeftInputPin;            /*!< Input Pin for indicator messages */
        cInputPin                m_oTurnSignalRightInputPin;            /*!< Input Pin for indicator messages */
        cInputPin                m_oHazardLightsInputPin;                     /*!< Input pin for the hazzard */
        cInputPin                m_oBrakeLightInputPin;                /*!< Input Pin for brake messages */
        cInputPin                m_oSteerInputPin;                   /*!< Input Pin for steer angle messages; values must be between 0...180 */
        cInputPin                m_oWatchdogInputPin;                /*!< Input Pin for watch dog flags */
        cInputPin                m_oEmergencyStopInputPin;           /*!< Input Pin for emergency stop */  

        
        /*!processes the packages for the lights
        @param pMediaSample the incoming mediasample containing the data
        @param ui8LightID the id of the selecet light
        */
        tResult ProcessLights(IMediaSample* pMediaSample, tUInt8 ui8LightID);
        
        /*! processes the not aus data
        @param pMediaSample the incoming mediasample containing the data
        */
        tResult ProcessEmergencyStop(IMediaSample* pMediaSample);
        
        /*! processes the watchdog data
        @param pMediaSample the incoming mediasample containing the data
        */
        tResult ProcessWatchdog(IMediaSample* pMediaSample);
        
        /*! processes the general actuator value
        @param chID the ID of the signal to be transmitted to the arduino
        @param pMediaSample the incoming mediasample containing the data
        */    
        tResult ProcessActuatorValue(IMediaSample* pMediaSample, tUInt8 chID);

        
        /*! builds the arduino frame to be transmitted to the arduino
        @param chID the ID of the signal to be transmitted to the arduino
        @param i8DataLength length of the data package
        @param pchData pointer to the data package
        */
        tResult BuildArduinoFrame(tUInt8 chID, tUInt8 i8DataLength, tUInt8 *pchData);
    
    public:
        cArduinoAktors(const tChar* __info);
        virtual ~cArduinoAktors();

    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    private:
        /*! creates all the output pins
        */
        tResult CreateOutputPins(__exception = NULL);
        
        /*! creates all the input pins
        */
        tResult CreateInputPins(__exception = NULL);
        
        /*! Coder Descriptor for the output pin */        
        cObjectPtr<IMediaTypeDescription> m_pCoderDescArduinoData;      /*!< descriptor for output data*/
        
        /*! Coder Descriptor for the input pin*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescEmergencyStop;           /*!< descriptor for not aus data*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;      /*!< descriptor for input data*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignalInput;  /*!< descriptor for tBool input data*/
        /*! if debug console output is enabled */
        tBool m_bDebugModeEnabled;
        /*! if watchdog debug to console is enabled*/
        tBool m_bDebugModeEnabledWatchdog;
        /*! if emergency stop was requested the watchdog has to be blocked to disable reactivation of the arduino */
        tBool m_bEmergencyStopBlock;
        
        /*! critical section for the processing of the  actuator samples */
        cCriticalSection        m_oActuatorCritSection;
        /*! critical section for the processing of the lights samples */
        cCriticalSection        m_oLightsCritSection;
  	/*! critical section for the processing of the watchdog samples */
        cCriticalSection        m_oWatchDogCritSection;
        /*! critical section for the transmitting of output samples */
        cCriticalSection        m_oTransmitCritSection;
        
};


//*************************************************************************************************

#endif // _AKTORS_FILTER_H_
